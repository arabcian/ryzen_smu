/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2020 Leonardo Gates <leogatesx9r@protonmail.com> */
/* Ryzen SMU Root Complex Communication */

#include <asm/cpuid/api.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/string.h>
#include <linux/time.h>
#include <linux/types.h>

#include "smu.h"

static struct {
  enum smu_processor_codename codename;

  // Optional RSMU mailbox addresses.
  u32 addr_rsmu_mb_cmd;
  u32 addr_rsmu_mb_rsp;
  u32 addr_rsmu_mb_args;

  // Mandatory MP1 mailbox addresses.
  enum smu_if_version mp1_if_ver;
  u32 addr_mp1_mb_cmd;
  u32 addr_mp1_mb_rsp;
  u32 addr_mp1_mb_args;

  u32 addr_hsmp_mb_cmd;
  u32 addr_hsmp_mb_rsp;
  u32 addr_hsmp_mb_args;

  // Optional PM table information.
  u64 pm_dram_base;
  u32 pm_dram_base_alt;
  u32 pm_dram_map_size;
  u32 pm_dram_map_size_alt;

  // Internal tracker to determine the minimum interval required to
  //  refresh the metrics table.
  //
  // N.B. This must be unsigned long, not u32: time_after() compares against
  //  the full-width jiffies counter and a truncated copy breaks the
  //  comparison every 2^32 ticks (and immediately after boot, since jiffies
  //  starts at INITIAL_JIFFIES == -5 minutes).
  unsigned long pm_jiffies;
  bool pm_jiffies_valid;

  // Virtual addresses mapped to the physical DRAM bases of the PM table.
  // Obtained via memremap(), NOT ioremap(): the PM table lives in ordinary
  //  (firmware-reserved) system DRAM, so these are normal kernel pointers.
  u8 *pm_table_virt_addr;
  u8 *pm_table_virt_addr_alt;

  // Byte length actually passed to memremap(), so it can be re-validated.
  size_t pm_table_mapped_size;
} g_smu = {
    .codename = CODENAME_UNDEFINED,

    .addr_rsmu_mb_cmd = 0,
    .addr_rsmu_mb_rsp = 0,
    .addr_rsmu_mb_args = 0,

    .mp1_if_ver = IF_VERSION_COUNT,
    .addr_mp1_mb_cmd = 0,
    .addr_mp1_mb_rsp = 0,
    .addr_mp1_mb_args = 0,

    .addr_hsmp_mb_cmd = 0,
    .addr_hsmp_mb_rsp = 0,
    .addr_hsmp_mb_args = 0,

    .pm_dram_base = 0,
    .pm_dram_base_alt = 0,
    .pm_dram_map_size = 0,
    .pm_dram_map_size_alt = 0,
    .pm_jiffies = 0,
    .pm_jiffies_valid = false,

    .pm_table_virt_addr = NULL,
    .pm_table_virt_addr_alt = NULL,
    .pm_table_mapped_size = 0,
};

// Both mutexes are defined separately because the SMN address space can be used
//  independently from the SMU but the SMU requires access to the SMN to execute
//  commands.
static DEFINE_MUTEX(amd_pci_mutex);
static DEFINE_MUTEX(amd_smu_mutex);

// Serializes the PM table path: DRAM base discovery, memremap() of the table
//  and the copy out of it. Without this, two concurrent readers can race in
//  smu_read_pm_table() and leak an entire mapping (both observe
//  pm_table_virt_addr == NULL and both call memremap()).
static DEFINE_MUTEX(amd_pm_mutex);

int smu_smn_rw_address(struct pci_dev *dev, u32 address, u32 *value,
                       int write) {
  int err;

  // This may work differently for multi-NUMA systems.
  mutex_lock(&amd_pci_mutex);
  err = pci_write_config_dword(dev, SMU_PCI_ADDR_REG, address);

  if (!err) {
    err = (write ? pci_write_config_dword(dev, SMU_PCI_DATA_REG, *value)
                 : pci_read_config_dword(dev, SMU_PCI_DATA_REG, value));

    if (err)
      pr_warn("Error %s SMN address: 0x%x!\n", write ? "writing" : "reading",
              address);
  } else
    pr_warn("Error programming SMN address: 0x%x!\n", address);
  mutex_unlock(&amd_pci_mutex);

  return err;
}

enum smu_return_val smu_read_address(struct pci_dev *dev, u32 address,
                                     u32 *value) {
  return !smu_smn_rw_address(dev, address, value, 0) ? SMU_Return_OK
                                                     : SMU_Return_PCIFailed;
}

enum smu_return_val smu_write_address(struct pci_dev *dev, u32 address,
                                      u32 value) {
  return !smu_smn_rw_address(dev, address, &value, 1) ? SMU_Return_OK
                                                      : SMU_Return_PCIFailed;
}

void smu_args_init(smu_req_args_t *args, u32 value) {
  u32 i;

  args->args[0] = value;

  for (i = 1; i < SMU_REQ_MAX_ARGS; i++)
    args->args[i] = 0;
}

/**
 * Polls [rsp_addr] until it reads back non-zero, the PCI access fails or the
 * attempt budget is exhausted.
 *
 * The original implementation spun on raw PCI config cycles with preemption
 * enabled for up to smu_timeout_attempts (default 8192, max 32768)
 * iterations while holding both mailbox mutexes. On a wedged mailbox that
 * pins a CPU for tens of milliseconds and blocks every other SMU user.
 * We keep a short tight-spin window for the common (fast) case and then
 * fall back to sleeping between polls.
 */
static enum smu_return_val smu_poll_mailbox(struct pci_dev *dev, u32 rsp_addr,
                                            u32 *out, uint attempts) {
  u32 tmp;
  uint i;

  for (i = 0; i < attempts; i++) {
    if (smu_read_address(dev, rsp_addr, &tmp) != SMU_Return_OK)
      return SMU_Return_PCIFailed;

    if (tmp != 0) {
      *out = tmp;
      return SMU_Return_OK;
    }

    if (i < SMU_POLL_SPIN_ATTEMPTS)
      cpu_relax();
    else
      usleep_range(SMU_POLL_SLEEP_US_MIN, SMU_POLL_SLEEP_US_MAX);
  }

  *out = 0;
  return SMU_Return_CommandTimeout;
}

enum smu_return_val smu_send_command(struct pci_dev *dev, u32 op,
                                     smu_req_args_t *args,
                                     enum smu_mailbox mailbox) {
  u32 tmp, i, rsp_addr, args_addr, cmd_addr;
  enum smu_return_val ret;
  uint attempts;

  if (!dev || !args)
    return SMU_Return_InvalidArgument;

  // == Pick the correct mailbox address. ==
  switch (mailbox) {
  case MAILBOX_TYPE_RSMU:
    rsp_addr = g_smu.addr_rsmu_mb_rsp;
    cmd_addr = g_smu.addr_rsmu_mb_cmd;
    args_addr = g_smu.addr_rsmu_mb_args;
    break;
  case MAILBOX_TYPE_MP1:
    rsp_addr = g_smu.addr_mp1_mb_rsp;
    cmd_addr = g_smu.addr_mp1_mb_cmd;
    args_addr = g_smu.addr_mp1_mb_args;
    break;
  case MAILBOX_TYPE_HSMP:
    rsp_addr = g_smu.addr_hsmp_mb_rsp;
    cmd_addr = g_smu.addr_hsmp_mb_cmd;
    args_addr = g_smu.addr_hsmp_mb_args;
    break;
  default:
    return SMU_Return_Unsupported;
  }

  // == In the unlikely event a mailbox is undefined, don't even attempt to
  // execute. ==
  if (!rsp_addr || !cmd_addr || !args_addr)
    return SMU_Return_Unsupported;

  /*
   * Clamp at the point of use as well as at probe time. smu_timeout_attempts
   * is a writable module parameter, so a runtime write of 0 would otherwise
   * turn the old "retries--" underflow into a ~4 billion iteration spin.
   */
  attempts = smu_timeout_attempts;
  if (attempts > SMU_RETRIES_MAX)
    attempts = SMU_RETRIES_MAX;
  if (attempts < SMU_RETRIES_MIN)
    attempts = SMU_RETRIES_MIN;

  pr_debug(
      "SMU Service Request: ID(0x%x) Args(0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x)",
      op, args->s.arg0, args->s.arg1, args->s.arg2, args->s.arg3, args->s.arg4,
      args->s.arg5);

  mutex_lock(&amd_smu_mutex);

  // Step 1: Wait until the RSP register is non-zero, i.e. the mailbox is idle.
  ret = smu_poll_mailbox(dev, rsp_addr, &tmp, attempts);
  if (ret != SMU_Return_OK) {
    mutex_unlock(&amd_smu_mutex);

    if (ret == SMU_Return_PCIFailed)
      pr_warn("Failed to perform initial probe on SMU RSP!\n");
    else
      pr_debug("SMU Service Request Failed: Timeout on initial wait for "
               "mailbox availability.");

    return ret;
  }

  // Step 2: Write zero (0) to the RSP register.
  if (smu_write_address(dev, rsp_addr, 0) != SMU_Return_OK)
    goto pci_failed;

  // Step 3: Write the argument(s) into the argument register(s).
  for (i = 0; i < SMU_REQ_MAX_ARGS; i++)
    if (smu_write_address(dev, args_addr + (i * 4), args->args[i]) !=
        SMU_Return_OK)
      goto pci_failed;

  // Step 4: Write the message Id into the Message ID register.
  if (smu_write_address(dev, cmd_addr, op) != SMU_Return_OK)
    goto pci_failed;

  /*
   * Step 5: Wait until the Response register is non-zero.
   *
   * N.B. The attempt budget is deliberately re-armed here. The original code
   * shared a single "retries" counter across step 1 and step 5, so a slow
   * mailbox handover left almost no budget for the command itself and
   * produced spurious timeouts.
   */
  ret = smu_poll_mailbox(dev, rsp_addr, &tmp, attempts);
  if (ret != SMU_Return_OK) {
    mutex_unlock(&amd_smu_mutex);

    if (ret == SMU_Return_PCIFailed) {
      pr_warn("Failed to perform probe on SMU RSP!\n");
      return SMU_Return_PCIFailed;
    }

    pr_debug("SMU Service Request Failed: Timeout on command (0x%x) after %u "
             "attempts.",
             op, attempts);
    return SMU_Return_CommandTimeout;
  }

  /*
   * Step 6: The SMU has answered. Anything other than OK is a failure.
   *
   * N.B. The original condition was "if (tmp != SMU_Return_OK && !retries)",
   * which meant that whenever the mailbox answered quickly with an error
   * (Failed / UnknownCmd / RejectedPrereq / RejectedBusy) the function fell
   * through and returned SMU_Return_OK, reporting a rejected command as a
   * success and handing back stale argument registers.
   */
  if (tmp != SMU_Return_OK) {
    mutex_unlock(&amd_smu_mutex);
    pr_debug("SMU Service Request Failed: command (0x%x) returned %Xh.", op,
             tmp);
    return (enum smu_return_val)tmp;
  }

  // Step 7: If a return argument is expected, the Argument register may be read
  //  at this time.
  for (i = 0; i < SMU_REQ_MAX_ARGS; i++)
    if (smu_read_address(dev, args_addr + (i * 4), &args->args[i]) !=
        SMU_Return_OK)
      pr_warn("Failed to fetch SMU ARG [%d]!\n", i);

  mutex_unlock(&amd_smu_mutex);

  pr_debug(
      "SMU Service Response: ID(0x%x) Args(0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x)",
      op, args->s.arg0, args->s.arg1, args->s.arg2, args->s.arg3, args->s.arg4,
      args->s.arg5);

  return SMU_Return_OK;

pci_failed:
  mutex_unlock(&amd_smu_mutex);
  pr_warn("Failed to program the SMU mailbox for command 0x%x!\n", op);
  return SMU_Return_PCIFailed;
}

int smu_resolve_cpu_class(struct pci_dev *dev) {
  u32 cpuid, cpu_family, cpu_model, stepping, pkg_type;

  // https://en.wikichip.org/wiki/amd/cpuid
  // Res. + ExtFamily + ExtModel + Res. + BaseFamily + BaseModel + Stepping
  // See: CPUID_Fn00000001_EAX
  cpuid = cpuid_eax(0x00000001);

  cpu_family = ((cpuid & 0xf00) >> 8) + ((cpuid & 0xff00000) >> 20);
  cpu_model = ((cpuid & 0xf0000) >> 12) + ((cpuid & 0xf0) >> 4);
  stepping = cpuid & 0xf;

  // Combines "PkgType" and "Reserved"
  // See: CPUID_Fn80000001_EBX
  pkg_type = cpuid_ebx(0x80000001) >> 28;

  pr_info("CPUID: family 0x%X, model 0x%X, stepping 0x%X, package 0x%X",
          cpu_family, cpu_model, stepping, pkg_type);

  // Zen / Zen+ / Zen2
  if (cpu_family == 0x17) {
    switch (cpu_model) {
    case 0x01:
      if (pkg_type == 7)
        g_smu.codename = CODENAME_THREADRIPPER;
      else if (pkg_type == 4)
        g_smu.codename = CODENAME_NAPLES;
      else
        g_smu.codename = CODENAME_SUMMITRIDGE;
      break;
    case 0x08:
      if (pkg_type == 7 || pkg_type == 4)
        g_smu.codename = CODENAME_COLFAX;
      else
        g_smu.codename = CODENAME_PINNACLERIDGE;
      break;
    case 0x11:
      g_smu.codename = CODENAME_RAVENRIDGE;
      break;
    case 0x18:
      if (pkg_type == 2)
        g_smu.codename = CODENAME_RAVENRIDGE2;
      else
        g_smu.codename = CODENAME_PICASSO;
      break;
    case 0x20:
      g_smu.codename = CODENAME_DALI;
      break;
    case 0x31:
      g_smu.codename = CODENAME_CASTLEPEAK;
      break;
    case 0x60:
      g_smu.codename = CODENAME_RENOIR;
      break;
    case 0x68:
      g_smu.codename = CODENAME_LUCIENNE;
      break;
    case 0x71:
      g_smu.codename = CODENAME_MATISSE;
      break;
    case 0x90:
      g_smu.codename = CODENAME_VANGOGH;
      break;
    default:
      pr_err(
          "CPUID: Unknown Zen/Zen+/Zen2 processor model: 0x%X (CPUID: 0x%08X)",
          cpu_model, cpuid);
      return -2;
    }
    return 0;
  }

  // Zen3 / Zen4
  // At least from Zen3 onward AMD reserves 16 model IDs per generation
  // Chagall: 0x00-0x0F, Stormpeak: 0x10-0x1f, etc...
  // Ryzen Master uses this full reserved range to identify and probe CPUs
  // unlike us
  else if (cpu_family == 0x19) {
    switch (cpu_model) {
    case 0x01:
      g_smu.codename = CODENAME_MILAN;
      break;
    case 0x08:
      g_smu.codename = CODENAME_CHAGALL;
      break;
    case 0x18:
      g_smu.codename = CODENAME_STORMPEAK;
      break;
    case 0x20:
    case 0x21:
      g_smu.codename = CODENAME_VERMEER;
      break;
    case 0x40:
    case 0x44:
      g_smu.codename = CODENAME_REMBRANDT;
      break;
    case 0x50:
      g_smu.codename = CODENAME_CEZANNE;
      break;
    case 0x61:
      g_smu.codename = CODENAME_RAPHAEL;
      break;
    case 0x74:
      g_smu.codename = CODENAME_PHOENIX;
      break;
    case 0x75:
      g_smu.codename = CODENAME_HAWKPOINT;
      break;
    default:
      pr_err("CPUID: Unknown Zen3/4 processor model: 0x%X (CPUID: 0x%08X)",
             cpu_model, cpuid);
      return -2;
    }
    return 0;
  }

  // Zen 5
  else if (cpu_family == 0x1a) {
    switch (cpu_model) {
    case 0x24: // Strix Point (Ryzen AI 9 HX 370, family 0x1A model 0x24)
      g_smu.codename = CODENAME_STRIXPOINT;
      break;
    case 0x44:
      g_smu.codename = CODENAME_GRANITERIDGE;
      break;
    case 0x60: //Strixpoint Ryzen AI 350
      g_smu.codename = CODENAME_STRIXPOINT;
      break;
    case 0x70: // Strix Halo (AI MAX+ 395)
      g_smu.codename = CODENAME_STRIXHALO;
      break;
    default:
      pr_err("CPUID: Unknown Zen5/6 processor model: 0x%X (CPUID: 0x%08X)",
             cpu_model, cpuid);
      return -2;
    }
    return 0;
  }

  else {
    pr_err("CPUID: failed to detect Zen processor family "
           "(%Xh).",
           cpu_family);
    return -1;
  }
}

int smu_init(struct pci_dev *dev) {
  // This really should never be called twice however in case it is, consider it
  // initialized.
  if (g_smu.codename != CODENAME_UNDEFINED)
    return 0;

  if (smu_resolve_cpu_class(dev))
    return -ENODEV;

  // Detect RSMU mailbox address.
  switch (g_smu.codename) {
  case CODENAME_CASTLEPEAK:
  case CODENAME_MATISSE:
  case CODENAME_VERMEER:
  case CODENAME_MILAN:
  case CODENAME_CHAGALL:
  case CODENAME_RAPHAEL:
  case CODENAME_GRANITERIDGE:
  case CODENAME_STORMPEAK:
    g_smu.addr_rsmu_mb_cmd = 0x3B10524;
    g_smu.addr_rsmu_mb_rsp = 0x3B10570;
    g_smu.addr_rsmu_mb_args = 0x3B10A40;
    goto LOG_RSMU;
  case CODENAME_COLFAX:
  case CODENAME_NAPLES:
  case CODENAME_SUMMITRIDGE:
  case CODENAME_THREADRIPPER:
  case CODENAME_PINNACLERIDGE:
    g_smu.addr_rsmu_mb_cmd = 0x3B1051C;
    g_smu.addr_rsmu_mb_rsp = 0x3B10568;
    g_smu.addr_rsmu_mb_args = 0x3B10590;
    goto LOG_RSMU;
  case CODENAME_RENOIR:
  case CODENAME_LUCIENNE:
  case CODENAME_PICASSO:
  case CODENAME_CEZANNE:
  case CODENAME_RAVENRIDGE:
  case CODENAME_RAVENRIDGE2:
  case CODENAME_DALI:
  case CODENAME_REMBRANDT:
  case CODENAME_PHOENIX:
  case CODENAME_STRIXPOINT:
  case CODENAME_HAWKPOINT:
  case CODENAME_STRIXHALO:
    g_smu.addr_rsmu_mb_cmd = 0x3B10A20;
    g_smu.addr_rsmu_mb_rsp = 0x3B10A80;
    g_smu.addr_rsmu_mb_args = 0x3B10A88;
    goto LOG_RSMU;
  case CODENAME_VANGOGH:
    pr_debug("RSMU Mailbox: Not supported or unknown, disabling use.");
    goto MP1_DETECT;
  default:
    pr_err("Unknown processor codename: %d", g_smu.codename);
    return -ENODEV;
  }

LOG_RSMU:
  pr_debug("RSMU Mailbox: (cmd: 0x%X, rsp: 0x%X, args: 0x%X)",
           g_smu.addr_rsmu_mb_cmd, g_smu.addr_rsmu_mb_rsp,
           g_smu.addr_rsmu_mb_args);

  // Detect HSMP mailbox address.
  switch (g_smu.codename) {
  case CODENAME_CASTLEPEAK:
  case CODENAME_MATISSE:
  case CODENAME_VERMEER:
  case CODENAME_MILAN:
  case CODENAME_CHAGALL:
  case CODENAME_RAPHAEL:
  case CODENAME_GRANITERIDGE:
  case CODENAME_STORMPEAK:
    g_smu.addr_hsmp_mb_cmd = 0x3B10534;
    g_smu.addr_hsmp_mb_rsp = 0x3B10980;
    g_smu.addr_hsmp_mb_args = 0x3B109E0;
    goto LOG_HSMP;
  case CODENAME_CEZANNE:
  case CODENAME_COLFAX:
  case CODENAME_NAPLES:
  case CODENAME_SUMMITRIDGE:
  case CODENAME_THREADRIPPER:
  case CODENAME_PINNACLERIDGE:
  case CODENAME_RENOIR:
  case CODENAME_LUCIENNE:
  case CODENAME_PICASSO:
  case CODENAME_RAVENRIDGE:
  case CODENAME_RAVENRIDGE2:
  case CODENAME_DALI:
  case CODENAME_VANGOGH:
  case CODENAME_REMBRANDT:
  case CODENAME_PHOENIX:
  case CODENAME_STRIXPOINT:
  case CODENAME_HAWKPOINT:
  case CODENAME_STRIXHALO:
    goto MP1_DETECT;
  default:
    pr_err("Unknown processor codename: %d", g_smu.codename);
    return -ENODEV;
  }

LOG_HSMP:
  pr_debug("HSMP Mailbox: (cmd: 0x%X, rsp: 0x%X, args: 0x%X)",
           g_smu.addr_hsmp_mb_cmd, g_smu.addr_hsmp_mb_rsp,
           g_smu.addr_hsmp_mb_args);

MP1_DETECT:
  // Detect MP1 SMU mailbox address.
  switch (g_smu.codename) {
  case CODENAME_COLFAX:
  case CODENAME_NAPLES:
  case CODENAME_SUMMITRIDGE:
  case CODENAME_THREADRIPPER:
  case CODENAME_PINNACLERIDGE:
    g_smu.mp1_if_ver = IF_VERSION_9;
    g_smu.addr_mp1_mb_cmd = 0x3B10528;
    g_smu.addr_mp1_mb_rsp = 0x3B10564;
    g_smu.addr_mp1_mb_args = 0x3B10598;
    break;
  case CODENAME_PICASSO:
  case CODENAME_RAVENRIDGE:
  case CODENAME_RAVENRIDGE2:
  case CODENAME_DALI:
    g_smu.mp1_if_ver = IF_VERSION_10;
    g_smu.addr_mp1_mb_cmd = 0x3B10528;
    g_smu.addr_mp1_mb_rsp = 0x3B10564;
    g_smu.addr_mp1_mb_args = 0x3B10998;
    break;
  case CODENAME_MATISSE:
  case CODENAME_VERMEER:
  case CODENAME_CASTLEPEAK:
  case CODENAME_MILAN:
  case CODENAME_CHAGALL:
  case CODENAME_RAPHAEL:
  case CODENAME_GRANITERIDGE:
  case CODENAME_STORMPEAK:
    g_smu.mp1_if_ver = IF_VERSION_11;
    g_smu.addr_mp1_mb_cmd = 0x3B10530;
    g_smu.addr_mp1_mb_rsp = 0x3B1057C;
    g_smu.addr_mp1_mb_args = 0x3B109C4;
    break;
  case CODENAME_RENOIR:
  case CODENAME_LUCIENNE:
  case CODENAME_CEZANNE:
    g_smu.mp1_if_ver = IF_VERSION_12;
    g_smu.addr_mp1_mb_cmd = 0x3B10528;
    g_smu.addr_mp1_mb_rsp = 0x3B10564;
    g_smu.addr_mp1_mb_args = 0x3B10998;
    break;
  case CODENAME_VANGOGH:
  case CODENAME_REMBRANDT:
  case CODENAME_PHOENIX:
  case CODENAME_HAWKPOINT:
    g_smu.mp1_if_ver = IF_VERSION_13;
    g_smu.addr_mp1_mb_cmd = 0x3B10528;
    g_smu.addr_mp1_mb_rsp = 0x3B10578;
    g_smu.addr_mp1_mb_args = 0x3B10998;
    break;
  case CODENAME_STRIXPOINT:
  case CODENAME_STRIXHALO:
    g_smu.mp1_if_ver = IF_VERSION_13;
    g_smu.addr_mp1_mb_cmd = 0x3b10928;
    g_smu.addr_mp1_mb_rsp = 0x3b10978;
    g_smu.addr_mp1_mb_args = 0x3b10998;
    break;
  default:
    pr_err("Unknown processor codename: %d", g_smu.codename);
    return -ENODEV;
  }

  pr_debug("MP1 Mailbox: (cmd: 0x%X, rsp: 0x%X, args: 0x%X)",
           g_smu.addr_mp1_mb_cmd, g_smu.addr_mp1_mb_rsp,
           g_smu.addr_mp1_mb_args);

  pr_info("Family Codename: %s", getCodeName(g_smu.codename));

  return 0;
}

const char *getCodeName(enum smu_processor_codename codename) {
  switch (codename) {
  case CODENAME_COLFAX:
    return "Colfax";
  case CODENAME_RENOIR:
    return "Renoir";
  case CODENAME_PICASSO:
    return "Picasso";
  case CODENAME_MATISSE:
    return "Matisse";
  case CODENAME_THREADRIPPER:
    return "ThreadRipper";
  case CODENAME_CASTLEPEAK:
    return "CastelPeak";
  case CODENAME_RAVENRIDGE:
    return "RavenRidge";
  case CODENAME_RAVENRIDGE2:
    return "RavenRidge2";
  case CODENAME_SUMMITRIDGE:
    return "SummitRidge";
  case CODENAME_PINNACLERIDGE:
    return "PinnacleRidge";
  case CODENAME_REMBRANDT:
    return "Rembrandt";
  case CODENAME_VERMEER:
    return "Vermeer";
  case CODENAME_VANGOGH:
    return "VanGogh";
  case CODENAME_CEZANNE:
    return "Cezanne";
  case CODENAME_MILAN:
    return "Milan";
  case CODENAME_DALI:
    return "Dali";
  case CODENAME_LUCIENNE:
    return "Lucienne";
  case CODENAME_NAPLES:
    return "Naples";
  case CODENAME_CHAGALL:
    return "Chagall";
  case CODENAME_RAPHAEL:
    return "Raphael";
  case CODENAME_GRANITERIDGE:
    return "GraniteRidge";
  case CODENAME_PHOENIX:
    return "Phoenix";
  case CODENAME_STRIXPOINT:
    return "Strix Point";
  case CODENAME_HAWKPOINT:
    return "Hawk Point";
  case CODENAME_STORMPEAK:
    return "Storm Peak";
  case CODENAME_STRIXHALO:
    return "Strix Halo";
  default:
    return "Undefined";
  }
}
void smu_cleanup(void) {
  mutex_lock(&amd_pm_mutex);

  // Unmap DRAM Base if required after SMU use.
  if (g_smu.pm_table_virt_addr) {
    memunmap(g_smu.pm_table_virt_addr);
    g_smu.pm_table_virt_addr = NULL;
  }

  if (g_smu.pm_table_virt_addr_alt) {
    memunmap(g_smu.pm_table_virt_addr_alt);
    g_smu.pm_table_virt_addr_alt = NULL;
  }

  /*
   * Drop every piece of derived PM table state as well. The original code
   * only cleared the mappings, so a probe -> remove -> probe cycle within a
   * single module load (PCI hotplug, or a second matching root complex)
   * would keep a stale DRAM base and table size around and skip
   * re-discovery entirely.
   */
  g_smu.pm_dram_base = 0;
  g_smu.pm_dram_base_alt = 0;
  g_smu.pm_dram_map_size = 0;
  g_smu.pm_dram_map_size_alt = 0;
  g_smu.pm_table_mapped_size = 0;
  g_smu.pm_jiffies = 0;
  g_smu.pm_jiffies_valid = false;

  mutex_unlock(&amd_pm_mutex);

  // Set SMU state to uninitialized, requiring a call to smu_init() again.
  g_smu.codename = CODENAME_UNDEFINED;
}

size_t smu_get_pm_table_size(void) { return g_smu.pm_dram_map_size; }

enum smu_processor_codename smu_get_codename(void) { return g_smu.codename; }

u32 smu_get_version(struct pci_dev *dev, enum smu_mailbox mb) {
  smu_req_args_t args;
  u32 ret;

  // First value is always 1.
  smu_args_init(&args, 1);

  // OP 0x02 is consistent with all platforms meaning
  //  it can be used directly.
  ret = smu_send_command(dev, 0x02, &args, mb);
  if (ret != SMU_Return_OK)
    return ret;

  return args.s.arg0;
}

enum smu_if_version smu_get_mp1_if_version(void) { return g_smu.mp1_if_ver; }

enum smu_return_val smu_get_dram_base_address(struct pci_dev *dev, u64 *base) {
  u32 fn[3] = {0, 0, 0}, parts[2] = {0, 0};
  enum smu_return_val ret;
  smu_req_args_t args;

  const enum smu_mailbox type = MAILBOX_TYPE_RSMU;

  if (!base)
    return SMU_Return_InvalidArgument;

  *base = 0;
  smu_args_init(&args, 0);

  switch (g_smu.codename) {
  case CODENAME_NAPLES:
  case CODENAME_SUMMITRIDGE:
  case CODENAME_THREADRIPPER:
    fn[0] = 0xa;
    goto BASE_ADDR_CLASS_1;
  case CODENAME_VERMEER:
  case CODENAME_MATISSE:
  case CODENAME_CASTLEPEAK:
  case CODENAME_MILAN:
  case CODENAME_CHAGALL:
    fn[0] = 0x06;
    goto BASE_ADDR_CLASS_1;
  case CODENAME_RAPHAEL:
  case CODENAME_GRANITERIDGE:
  case CODENAME_STORMPEAK:
    fn[0] = 0x04;
    goto BASE_ADDR_CLASS_1;
  case CODENAME_RENOIR:
  case CODENAME_LUCIENNE:
  case CODENAME_CEZANNE:
  case CODENAME_REMBRANDT:
  case CODENAME_PHOENIX:
  case CODENAME_STRIXPOINT:
  case CODENAME_STRIXHALO:
  case CODENAME_HAWKPOINT:
    fn[0] = 0x66;
    goto BASE_ADDR_CLASS_1;
  case CODENAME_COLFAX:
  case CODENAME_PINNACLERIDGE:
    fn[0] = 0x0b;
    fn[1] = 0x0c;
    goto BASE_ADDR_CLASS_2;
  case CODENAME_DALI:
  case CODENAME_PICASSO:
  case CODENAME_RAVENRIDGE:
  case CODENAME_RAVENRIDGE2:
    fn[0] = 0x0a;
    fn[1] = 0x3d;
    fn[2] = 0x0b;
    goto BASE_ADDR_CLASS_3;
  default:
    return SMU_Return_Unsupported;
  }

BASE_ADDR_CLASS_1:
  args.s.arg0 = args.s.arg1 = 1;
  ret = smu_send_command(dev, fn[0], &args, type);
  if (ret != SMU_Return_OK)
    return ret;

  *base = (u64)args.s.arg0 | ((u64)args.s.arg1 << 32);
  return SMU_Return_OK;

BASE_ADDR_CLASS_2:
  ret = smu_send_command(dev, fn[0], &args, type);
  if (ret != SMU_Return_OK)
    return ret;

  smu_args_init(&args, 0);
  ret = smu_send_command(dev, fn[1], &args, type);
  if (ret != SMU_Return_OK)
    return ret;

  *base = args.s.arg0;
  return SMU_Return_OK;

BASE_ADDR_CLASS_3:
  // == Part 1 ==
  args.s.arg0 = 3;
  ret = smu_send_command(dev, fn[0], &args, type);
  if (ret != SMU_Return_OK)
    return ret;

  smu_args_init(&args, 3);
  ret = smu_send_command(dev, fn[2], &args, type);
  if (ret != SMU_Return_OK)
    return ret;

  // 1st Base.
  parts[0] = args.s.arg0;
  // == Part 1 End ==

  // == Part 2 ==
  smu_args_init(&args, 3);
  ret = smu_send_command(dev, fn[1], &args, type);
  if (ret != SMU_Return_OK)
    return ret;

  smu_args_init(&args, 5);
  ret = smu_send_command(dev, fn[0], &args, type);
  if (ret != SMU_Return_OK)
    return ret;

  smu_args_init(&args, 5);
  ret = smu_send_command(dev, fn[2], &args, type);
  if (ret != SMU_Return_OK)
    return ret;

  // 2nd base.
  parts[1] = args.s.arg0;
  // == Part 2 End ==

  *base = (u64)parts[1] << 32 | parts[0];
  return SMU_Return_OK;
}

enum smu_return_val smu_transfer_table_to_dram(struct pci_dev *dev) {
  smu_req_args_t args;
  u32 fn;

  /**
   * Probes (updates) the PM Table.
   * SMC Message corresponds to TransferTableSmu2Dram.
   * Physically mapped at the DRAM Base address(es).
   */

  // Arg[0] here specifies the PM table when set to 0.
  // For GPU ASICs, it seems there's more tables that can be found but for CPUs,
  //  it seems this value is ignored.
  smu_args_init(&args, 0);

  switch (g_smu.codename) {
  case CODENAME_SUMMITRIDGE:
  case CODENAME_THREADRIPPER:
  case CODENAME_NAPLES:
    fn = 0x0a;
    break;
  case CODENAME_CASTLEPEAK:
  case CODENAME_MATISSE:
  case CODENAME_VERMEER:
  case CODENAME_MILAN:
  case CODENAME_CHAGALL:
    fn = 0x05;
    break;
  case CODENAME_RAPHAEL:
  case CODENAME_GRANITERIDGE:
  case CODENAME_STORMPEAK:
    fn = 0x03;
    break;
  case CODENAME_CEZANNE:
    fn = 0x65;
    break;
  case CODENAME_RENOIR:
  case CODENAME_LUCIENNE:
  case CODENAME_REMBRANDT:
  case CODENAME_PHOENIX:
  case CODENAME_STRIXPOINT:
  case CODENAME_STRIXHALO:
  case CODENAME_HAWKPOINT:
    args.s.arg0 = 3;
    fn = 0x65;
    break;
  case CODENAME_COLFAX:
  case CODENAME_PINNACLERIDGE:
  case CODENAME_PICASSO:
  case CODENAME_RAVENRIDGE:
  case CODENAME_RAVENRIDGE2:
    args.s.arg0 = 3;
    fn = 0x3d;
    break;
  default:
    return SMU_Return_Unsupported;
  }

  return smu_send_command(dev, fn, &args, MAILBOX_TYPE_RSMU);
}

enum smu_return_val smu_transfer_2nd_table_to_dram(struct pci_dev *dev) {
  smu_req_args_t args;
  u32 fn;

  /**
   * Probes (updates) the secondary PM Table.
   * SMC Message corresponds to TransferTableSmu2Dram.
   * Physically mapped at the DRAM Base address(es).
   */

  // Arg[0] here specifies the PM table when set to 0.
  // For GPU ASICs, it seems there's more tables that can be found but for CPUs,
  //  it seems this value is ignored.
  smu_args_init(&args, 0);

  switch (g_smu.codename) {
  case CODENAME_COLFAX:
  case CODENAME_PINNACLERIDGE:
  case CODENAME_PICASSO:
  case CODENAME_RAVENRIDGE:
  case CODENAME_RAVENRIDGE2:
    args.s.arg0 = 5;
    fn = 0x3d;
    break;
  case CODENAME_SUMMITRIDGE:
  case CODENAME_THREADRIPPER:
  case CODENAME_NAPLES:
  case CODENAME_CASTLEPEAK:
  case CODENAME_MATISSE:
  case CODENAME_VERMEER:
  case CODENAME_MILAN:
  case CODENAME_CEZANNE:
  case CODENAME_RENOIR:
  case CODENAME_LUCIENNE:
  default:
    return SMU_Return_Unsupported;
  }

  return smu_send_command(dev, fn, &args, MAILBOX_TYPE_RSMU);
}

enum smu_return_val smu_get_pm_table_version(struct pci_dev *dev,
                                             u32 *version) {
  enum smu_return_val ret;
  smu_req_args_t args;
  u32 fn;

  /**
   * For some codenames, there are different PM tables for each chip.
   * SMC Message corresponds to TableVersionId.
   * Based on AGESA FW revision.
   */
  switch (g_smu.codename) {
  case CODENAME_RAVENRIDGE:
  case CODENAME_PICASSO:
    fn = 0x0c;
    break;
  case CODENAME_CASTLEPEAK:
  case CODENAME_MATISSE:
  case CODENAME_VERMEER:
  case CODENAME_MILAN:
  case CODENAME_CHAGALL:
    fn = 0x08;
    break;
  case CODENAME_RAPHAEL:
  case CODENAME_GRANITERIDGE:
  case CODENAME_STORMPEAK:
    fn = 0x05;
    break;
  case CODENAME_RENOIR:
  case CODENAME_LUCIENNE:
  case CODENAME_CEZANNE:
  case CODENAME_REMBRANDT:
  case CODENAME_PHOENIX:
  case CODENAME_STRIXPOINT:
  case CODENAME_STRIXHALO:
  case CODENAME_HAWKPOINT:
    fn = 0x06;
    break;
  default:
    return SMU_Return_Unsupported;
  }

  if (!version)
    return SMU_Return_InvalidArgument;

  smu_args_init(&args, 0);

  ret = smu_send_command(dev, fn, &args, MAILBOX_TYPE_RSMU);

  // Only publish the value on success; on failure args[] holds whatever the
  // mailbox last left behind and the caller would key a table size off it.
  if (ret == SMU_Return_OK)
    *version = args.s.arg0;

  return ret;
}

enum smu_return_val smu_update_pmtable_size(u32 version) {
  // These sizes are actually accurate and not just "guessed".
  // Source: Ryzen Master.
  switch (g_smu.codename) {
  case CODENAME_CASTLEPEAK:
  case CODENAME_MATISSE:
    switch (version) {
    case 0x240003:
      g_smu.pm_dram_map_size = 0x18AC;
      break;
    case 0x240503:
      g_smu.pm_dram_map_size = 0xD7C;
      break;
    case 0x240603:
      g_smu.pm_dram_map_size = 0xAB0;
      break;
    case 0x240703:
      g_smu.pm_dram_map_size = 0x7E4;
      break;
    case 0x240802:
      g_smu.pm_dram_map_size = 0x7E0;
      break;
    case 0x240803:
      g_smu.pm_dram_map_size = 0x7E4;
      break;
    case 0x240902:
      g_smu.pm_dram_map_size = 0x514;
      break;
    case 0x240903:
      g_smu.pm_dram_map_size = 0x518;
      break;
    default:
      goto UNKNOWN_PM_TABLE_VERSION;
    }
    break;
  case CODENAME_VERMEER:
  case CODENAME_CHAGALL:
    switch (version) {
    case 0x2D0803:
      g_smu.pm_dram_map_size = 0x894;
      break;
    case 0x2D0903:
      g_smu.pm_dram_map_size = 0x594;
      break;
    case 0x380005:
      g_smu.pm_dram_map_size = 0x1BB0;
      break;
    case 0x380505:
      g_smu.pm_dram_map_size = 0xF30;
      break;
    case 0x380605:
      g_smu.pm_dram_map_size = 0xC10;
      break;
    case 0x380705:
      g_smu.pm_dram_map_size = 0x8F0;
      break;
    case 0x380804:
      g_smu.pm_dram_map_size = 0x8A4;
      break;
    case 0x380805:
      g_smu.pm_dram_map_size = 0x8F0;
      break;
    case 0x380904:
      g_smu.pm_dram_map_size = 0x5A4;
      break;
    case 0x380905:
      g_smu.pm_dram_map_size = 0x5D0;
      break;
    default:
      goto UNKNOWN_PM_TABLE_VERSION;
    }
    break;
  case CODENAME_MILAN:
    switch (version) {
    case 0x2D0008: // Don't exist in RM.
      g_smu.pm_dram_map_size = 0x1AB0;
      break;
    default:
      goto UNKNOWN_PM_TABLE_VERSION;
    }
    break;
  case CODENAME_RENOIR:
  case CODENAME_LUCIENNE:
    switch (version) {
    case 0x370000:
      g_smu.pm_dram_map_size = 0x794;
      break;
    case 0x370001:
      g_smu.pm_dram_map_size = 0x884;
      break;
    case 0x370002:
      g_smu.pm_dram_map_size = 0x88C;
      break;
    case 0x370003:
      g_smu.pm_dram_map_size = 0x8AC;
      break;
    case 0x370005:
      g_smu.pm_dram_map_size = 0x8C8;
      break;
    default:
      goto UNKNOWN_PM_TABLE_VERSION;
    }
    break;
  case CODENAME_CEZANNE:
    switch (version) {
    case 0x400005:
      g_smu.pm_dram_map_size = 0x944;
      break;
    default:
      goto UNKNOWN_PM_TABLE_VERSION;
    }
    break;
  case CODENAME_REMBRANDT:
    switch (version) {
    case 0x450004:
      g_smu.pm_dram_map_size = 0xAA4;
      break;
    case 0x450005:
      g_smu.pm_dram_map_size = 0xAB0;
      break;
    default:
      goto UNKNOWN_PM_TABLE_VERSION;
    }
    break;
  case CODENAME_PICASSO:
  case CODENAME_RAVENRIDGE:
  case CODENAME_RAVENRIDGE2:
    // These codenames have two PM tables, a larger (primary) one and a smaller
    // one. The size is always fixed to 0x608 and 0xA4 bytes each. Source: Ryzen
    // Master.
    g_smu.pm_dram_map_size_alt = 0xA4;
    g_smu.pm_dram_map_size = 0x608 + g_smu.pm_dram_map_size_alt;

    // Split DRAM base into high/low values.
    g_smu.pm_dram_base_alt = g_smu.pm_dram_base >> 32;
    g_smu.pm_dram_base &= 0xFFFFFFFF;
    break;
  case CODENAME_RAPHAEL:
    switch (version) {
    case 0x000400: // Some ES-time table? Don't exist in RM.
      g_smu.pm_dram_map_size = 0x948;
      break;
    case 0x540000:
      g_smu.pm_dram_map_size = 0x828;
      break;
    case 0x540001:
      g_smu.pm_dram_map_size = 0x82C;
      break;
    case 0x540002:
      g_smu.pm_dram_map_size = 0x87C;
      break;
    case 0x540003:
      g_smu.pm_dram_map_size = 0x89C;
      break;
    case 0x540004:
      g_smu.pm_dram_map_size = 0x8BC;
      break;
    case 0x540005:
      g_smu.pm_dram_map_size = 0x8C8;
      break;
    case 0x540100:
      g_smu.pm_dram_map_size = 0x618;
      break;
    case 0x540101:
      g_smu.pm_dram_map_size = 0x61C;
      break;
    case 0x540102:
      g_smu.pm_dram_map_size = 0x66C;
      break;
    case 0x540103:
      g_smu.pm_dram_map_size = 0x68C;
      break;
    case 0x540104:
      g_smu.pm_dram_map_size = 0x6A8;
      break;
    case 0x540105:
      g_smu.pm_dram_map_size = 0x6B4;
      break;
    case 0x540108:
      g_smu.pm_dram_map_size = 0x6BC;
      break;
    case 0x540208:
      g_smu.pm_dram_map_size = 0x8D0;
      break;
    default:
      goto UNKNOWN_PM_TABLE_VERSION;
    }
    break;
  case CODENAME_GRANITERIDGE:
    switch (version) {
    case 0x620105:
      g_smu.pm_dram_map_size = 0x724;
      break;
    case 0x620205:
      g_smu.pm_dram_map_size = 0x994;
      break;
    default:
      goto UNKNOWN_PM_TABLE_VERSION;
    }
    break;
  case CODENAME_PHOENIX:
  case CODENAME_HAWKPOINT:
    switch (version) {
    case 0x4C0003:
      g_smu.pm_dram_map_size = 0xB18;
      break;
    case 0x4C0004:
      g_smu.pm_dram_map_size = 0xB1C;
      break;
    case 0x4C0005:
      g_smu.pm_dram_map_size = 0xAF8;
      break;
    case 0x4C0006:
      g_smu.pm_dram_map_size = 0xAFC;
      break;
    case 0x4C0007:
      g_smu.pm_dram_map_size = 0xB00;
      break;
    case 0x4C0008:
      g_smu.pm_dram_map_size = 0xAF0;
      break;
    case 0x4C0009:
      g_smu.pm_dram_map_size = 0xB00;
      break;
    default:
      goto UNKNOWN_PM_TABLE_VERSION;
    }
    break;
  case CODENAME_STRIXPOINT:
    switch (version) {
    case 0x5D0008:
    case 0x5D0009: // Ryzen PRO / Ryzen AI 9 HX 370 (Strix Point, verified pm_table_version=0x5D0009, pm_table_size=0xD54)
      g_smu.pm_dram_map_size = 0xD54;
      break;
    case 0x650007: // Ryzen AI 7 350 variant
      g_smu.pm_dram_map_size = 0xD54;
      break;
    default:
      goto UNKNOWN_PM_TABLE_VERSION;
    }
    break;
  case CODENAME_STRIXHALO:
    switch (version) {
    case 0x64020C:
      g_smu.pm_dram_map_size = 0xE50;
      break;
    default:
      goto UNKNOWN_PM_TABLE_VERSION;
    }
    break;
  case CODENAME_STORMPEAK:
    switch (version) {
    case 0x5C0002:
      g_smu.pm_dram_map_size = 0x1E3C;
      break;
    case 0x5C0003:
      g_smu.pm_dram_map_size = 0x1E48;
      break;
    case 0x5C0102:
      g_smu.pm_dram_map_size = 0x1A14;
      break;
    case 0x5C0103:
      g_smu.pm_dram_map_size = 0x1A20;
      break;
    case 0x5C0202:
      g_smu.pm_dram_map_size = 0x15EC;
      break;
    case 0x5C0203:
      g_smu.pm_dram_map_size = 0x15F8;
      break;
    case 0x5C0302:
      g_smu.pm_dram_map_size = 0xD9C;
      break;
    case 0x5C0303:
      g_smu.pm_dram_map_size = 0xDA8;
      break;
    case 0x5C0402:
      g_smu.pm_dram_map_size = 0x974;
      break;
    case 0x5C0403:
      g_smu.pm_dram_map_size = 0x980;
      break;
    default:
      goto UNKNOWN_PM_TABLE_VERSION;
    }
    break;
  default:
  UNKNOWN_PM_TABLE_VERSION:
    return SMU_Return_Unsupported;
  }

  /*
   * Hard backstop. Every consumer of pm_dram_map_size (including the
   * driver's kzalloc()'d PM_TABLE_MAX_SIZE staging buffer) assumes the
   * value fits. If a table definition is ever added that exceeds the cap,
   * refuse the feature instead of silently overflowing a heap allocation.
   */
  if (g_smu.pm_dram_map_size > PM_TABLE_MAX_SIZE ||
      g_smu.pm_dram_map_size_alt > g_smu.pm_dram_map_size) {
    pr_err("PM table size 0x%X (alt 0x%X) exceeds the driver limit of 0x%X -- "
           "disabling PM table support",
           g_smu.pm_dram_map_size, g_smu.pm_dram_map_size_alt,
           PM_TABLE_MAX_SIZE);
    g_smu.pm_dram_map_size = 0;
    g_smu.pm_dram_map_size_alt = 0;
    return SMU_Return_InsufficientSize;
  }

  return SMU_Return_OK;
}

/* Anything below this is firmware/low-memory, never a valid PM table base. */
#define SMU_PM_DRAM_BASE_MIN 0x100000ULL

/* Caller must hold amd_pm_mutex. */
static enum smu_return_val smu_resolve_pm_layout(struct pci_dev *dev) {
  enum smu_return_val ret;
  u32 version = 0;
  u64 base = 0;

  if (g_smu.pm_dram_base && g_smu.pm_dram_map_size)
    return SMU_Return_OK;

  ret = smu_get_dram_base_address(dev, &base);
  if (ret != SMU_Return_OK) {
    pr_err("Unable to receive the DRAM base address: %X", ret);
    return ret;
  }

  /*
   * Sanity-check the base before it ever reaches memremap(). The old code
   * overloaded the return value as both an address and an error code
   * ("if (base < 0xFF && base >= 0)", where the second half is always true
   * for a u64) and would happily map whatever the mailbox returned.
   */
  if (base < SMU_PM_DRAM_BASE_MIN) {
    pr_err("Refusing implausible PM table DRAM base: 0x%llX", base);
    return SMU_Return_MappedError;
  }

  g_smu.pm_dram_base = base;

  // These models require finding the PM table version to determine its size.
  switch (g_smu.codename) {
  case CODENAME_VERMEER:
  case CODENAME_MATISSE:
  case CODENAME_RAPHAEL:
  case CODENAME_GRANITERIDGE:
  case CODENAME_RENOIR:
  case CODENAME_LUCIENNE:
  case CODENAME_REMBRANDT:
  case CODENAME_PHOENIX:
  case CODENAME_STRIXPOINT:
  case CODENAME_STRIXHALO:
  case CODENAME_CEZANNE:
  case CODENAME_CHAGALL:
  case CODENAME_MILAN:
  case CODENAME_HAWKPOINT:
  case CODENAME_STORMPEAK:
    ret = smu_get_pm_table_version(dev, &version);
    if (ret != SMU_Return_OK) {
      pr_err("Failed to get PM Table version with error: %X\n", ret);
      goto fail;
    }
    break;
  default:
    // Fixed-size table for this codename; version stays 0 and is unused.
    break;
  }

  ret = smu_update_pmtable_size(version);
  if (ret != SMU_Return_OK) {
    pr_err("Unknown PM table version: 0x%08X", version);
    goto fail;
  }

  /*
   * Re-validate after smu_update_pmtable_size(): for Picasso / RavenRidge
   * it splits the 64-bit value into two independent 32-bit bases, so the
   * check above did not cover what actually gets mapped.
   */
  if (g_smu.pm_dram_base < SMU_PM_DRAM_BASE_MIN ||
      (g_smu.pm_dram_map_size_alt &&
       g_smu.pm_dram_base_alt < SMU_PM_DRAM_BASE_MIN)) {
    pr_err("Refusing implausible PM table DRAM base pair: 0x%llX / 0x%X",
           g_smu.pm_dram_base, g_smu.pm_dram_base_alt);
    ret = SMU_Return_MappedError;
    goto fail;
  }

  pr_debug("Determined PM mapping size as (%xh,%xh) bytes.",
           g_smu.pm_dram_map_size, g_smu.pm_dram_map_size_alt);

  return SMU_Return_OK;

fail:
  // Do not leave a half-resolved layout behind for the next caller.
  g_smu.pm_dram_base = 0;
  g_smu.pm_dram_base_alt = 0;
  g_smu.pm_dram_map_size = 0;
  g_smu.pm_dram_map_size_alt = 0;
  return ret;
}

/* Caller must hold amd_pm_mutex. */
static enum smu_return_val smu_map_pm_table(size_t size) {
  if (g_smu.pm_table_virt_addr)
    return SMU_Return_OK;

  /*
   * memremap() rather than ioremap_cache(): the PM table is ordinary
   * (firmware-reserved) system DRAM. ioremap() on RAM creates a second
   * mapping with potentially conflicting cache attributes and warns on
   * modern kernels; memremap(MEMREMAP_WB) reuses the linear mapping when
   * the range is System RAM and falls back to a cached ioremap otherwise.
   */
  g_smu.pm_table_virt_addr = memremap(g_smu.pm_dram_base, size, MEMREMAP_WB);
  if (!g_smu.pm_table_virt_addr) {
    pr_err("Failed to map DRAM base: %llX (0x%zX B)", g_smu.pm_dram_base, size);
    return SMU_Return_MappedError;
  }

  g_smu.pm_table_mapped_size = size;

  // In Picasso/RavenRidge 2, we map the secondary (high) address as well.
  if (g_smu.pm_dram_map_size_alt) {
    g_smu.pm_table_virt_addr_alt = memremap(
        g_smu.pm_dram_base_alt, g_smu.pm_dram_map_size_alt, MEMREMAP_WB);

    if (!g_smu.pm_table_virt_addr_alt) {
      pr_err("Failed to map DRAM alt base: %X (0x%X B)", g_smu.pm_dram_base_alt,
             g_smu.pm_dram_map_size_alt);

      // Roll the primary mapping back so the next call retries cleanly
      // instead of copying from a half-initialised state.
      memunmap(g_smu.pm_table_virt_addr);
      g_smu.pm_table_virt_addr = NULL;
      g_smu.pm_table_mapped_size = 0;
      return SMU_Return_MappedError;
    }
  }

  return SMU_Return_OK;
}

enum smu_return_val smu_read_pm_table(struct pci_dev *dev, unsigned char *dst,
                                      size_t *len) {
  enum smu_return_val ret;
  size_t size;

  if (!dev || !dst || !len)
    return SMU_Return_InvalidArgument;

  mutex_lock(&amd_pm_mutex);

  // The DRAM base does not change after boot meaning it only needs to be
  //  fetched once.
  ret = smu_resolve_pm_layout(dev);
  if (ret != SMU_Return_OK)
    goto out;

  // Validate output buffer size.
  // N.B. In the case of Picasso/RavenRidge 2, we include the secondary PM
  // Table size as well.
  if (*len < g_smu.pm_dram_map_size) {
    pr_warn("Insufficient buffer size for PM table read: %zu < %u", *len,
            g_smu.pm_dram_map_size);

    *len = g_smu.pm_dram_map_size;
    ret = SMU_Return_InsufficientSize;
    goto out;
  }

  // Clamp output size
  *len = g_smu.pm_dram_map_size;

  // Primary PM Table size
  size = (size_t)g_smu.pm_dram_map_size - g_smu.pm_dram_map_size_alt;

  // Check if we should tell the SMU to refresh the table via jiffies.
  // Use a minimum interval of 1 ms.
  if (!g_smu.pm_jiffies_valid ||
      time_after(jiffies, g_smu.pm_jiffies + msecs_to_jiffies(1))) {
    g_smu.pm_jiffies = jiffies;
    g_smu.pm_jiffies_valid = true;

    ret = smu_transfer_table_to_dram(dev);
    if (ret != SMU_Return_OK)
      goto out;

    if (g_smu.pm_dram_map_size_alt) {
      ret = smu_transfer_2nd_table_to_dram(dev);
      if (ret != SMU_Return_OK)
        goto out;
    }
  }

  // We only map the DRAM base(s) once for use.
  ret = smu_map_pm_table(size);
  if (ret != SMU_Return_OK)
    goto out;

  // Paranoia: never copy more than what was actually mapped.
  if (size > g_smu.pm_table_mapped_size) {
    ret = SMU_Return_InsufficientSize;
    goto out;
  }

  memcpy(dst, g_smu.pm_table_virt_addr, size);

  // Append secondary table if required.
  if (g_smu.pm_dram_map_size_alt && g_smu.pm_table_virt_addr_alt)
    memcpy(dst + size, g_smu.pm_table_virt_addr_alt,
           g_smu.pm_dram_map_size_alt);

  ret = SMU_Return_OK;

out:
  mutex_unlock(&amd_pm_mutex);
  return ret;
}
