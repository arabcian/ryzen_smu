/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2020 Leonardo Gates <leogatesx9r@protonmail.com> */
/* Ryzen SMU Command Driver */

#include <linux/capability.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/security.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/version.h>
#include <uapi/linux/stat.h>

#include "smu.h"

#ifndef KBUILD_MODNAME
    #define KBUILD_MODNAME "ryzen_smu"
#endif

MODULE_AUTHOR("Leonardo Gates <leogatesx9r@protonmail.com>");
MODULE_DESCRIPTION("AMD Ryzen SMU Command Driver");
MODULE_VERSION("0.1.8");
MODULE_LICENSE("GPL");

#define MSEC_TO_NSEC(x)                    ((x) * 1000000)


#define PCI_DEVICE_ID_AMD_17H_ROOT          0x1450
#define PCI_DEVICE_ID_AMD_17H_M10H_ROOT     0x15d0
#define PCI_DEVICE_ID_AMD_17H_M60H_ROOT     0x1630
#define PCI_DEVICE_ID_AMD_17H_M30H_ROOT     0x1480

// https://github.com/torvalds/linux/blob/master/arch/x86/kernel/amd_nb.c
#define PCI_DEVICE_ID_AMD_1AH_M00H_ROOT     0x153a
#define PCI_DEVICE_ID_AMD_1AH_M20H_ROOT     0x1507
#define PCI_DEVICE_ID_AMD_1AH_M60H_ROOT     0x1122
#define PCI_DEVICE_ID_AMD_17H_MA0H_ROOT     0x14b5
#define PCI_DEVICE_ID_AMD_19H_M10H_ROOT     0x14a4
#define PCI_DEVICE_ID_AMD_19H_M40H_ROOT     0x14b5
#define PCI_DEVICE_ID_AMD_19H_M60H_ROOT     0x14d8
#define PCI_DEVICE_ID_AMD_19H_M70H_ROOT     0x14e8
#define PCI_DEVICE_ID_AMD_MI200_ROOT        0x14bb
#define PCI_DEVICE_ID_AMD_MI300_ROOT        0x14f8

#define MAX_ATTRS_LEN                      12

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 19, 0)
    #error "Unsupported kernel version. Minimum: v4.19"
#endif

/* sysfs_emit() appeared in v5.10; fall back to the bounded scnprintf(). */
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
    #define sysfs_emit(buf, fmt, ...) scnprintf(buf, PAGE_SIZE, fmt, ##__VA_ARGS__)
#endif

/**
 * sysfs permissions.
 *
 * RSMU_MODE_RO     - pure informational values, no hardware transaction.
 * RSMU_MODE_RO_HW  - reading issues SMU mailbox commands, so it is not
 *                    exposed to unprivileged users (an unprivileged loop over
 *                    pm_table would otherwise hammer the SMU mailbox).
 * RSMU_MODE_RW     - arbitrary SMN / mailbox control. Root only, and the
 *                    store handlers additionally require CAP_SYS_RAWIO.
 *
 * If a non-root monitoring daemon needs pm_table, relax it from userspace
 * (udev/tmpfiles.d against /sys/kernel/ryzen_smu_drv/pm_table) rather than
 * widening the default here.
 */
#define RSMU_MODE_RO                       (S_IRUSR | S_IRGRP | S_IROTH)
#define RSMU_MODE_RO_HW                    (S_IRUSR | S_IRGRP)
#define RSMU_MODE_RW                       (S_IRUSR | S_IWUSR)

/*
 * struct bin_attribute callbacks gained const pointers in v6.15. Older
 * kernels expect the non-const prototype.
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 15, 0)
    #define RSMU_BIN_ATTR_CONST const
#else
    #define RSMU_BIN_ATTR_CONST
#endif

#define __RO_ATTR(attr) \
    static struct kobj_attribute dev_attr_##attr = \
        __ATTR(attr, RSMU_MODE_RO, attr##_show, attr_store_null);

#define __RW_ATTR(attr) \
    static struct kobj_attribute dev_attr_##attr = \
        __ATTR(attr, RSMU_MODE_RW, attr##_show, attr##_store);

static struct ryzen_smu_data {
    struct pci_dev*         device;
    struct kobject*         drv_kobj;

    /*
     * Serializes every sysfs entry point. smu_args and the shared PM table
     * staging buffer are global driver state; without this two concurrent
     * writers can interleave an smu_args_store() into another thread's
     * *_cmd_store(), sending a command with someone else's arguments.
     */
    struct mutex            lock;

    char                    smu_version[64];
    smu_req_args_t          smu_args;
    u32                     smu_rsp;

    u32                     smn_result;

    u8*                     pm_table;
    u32                     pm_table_version;
    size_t                  pm_table_read_size;

    bool                    pm_table_enabled;
    bool                    sysfs_registered;
    bool                    pm_table_bin_registered;
} g_driver = {
    .device               = NULL,

    .drv_kobj             = NULL,

    .smu_version          = { 0 },
    .smu_args             = { .args = { 0, 0, 0, 0, 0, 0 } },
    .smu_rsp              = SMU_Return_OK,

    .smn_result           = 0,

    .pm_table             = NULL,
    .pm_table_version     = 0,
    .pm_table_read_size   = PM_TABLE_MAX_SIZE,

    .pm_table_enabled     = false,
    .sysfs_registered     = false,
    .pm_table_bin_registered = false,
};

/* Set once a device has been bound; guards against a second probe. */
static bool g_probed;

/* SMU Command Parameters. */
uint smu_timeout_attempts = 8192;

/**
 * Gate for every operation that pokes the hardware.
 *
 * The SMN read/write and mailbox attributes amount to arbitrary access to the
 * processor's internal register space, which is at least as powerful as
 * /dev/mem. File permissions alone are not the right control for that:
 *  - CAP_SYS_RAWIO is the capability the kernel uses for this class of access,
 *    and it is namespace-aware (root inside a user namespace does not hold it
 *    against the init user namespace).
 *  - Kernel lockdown exists precisely to block this under Secure Boot;
 *    without the check this module is a trivial lockdown bypass.
 */
static int ryzen_smu_check_privileged(void)
{
    if (!capable(CAP_SYS_RAWIO))
        return -EPERM;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
    {
        int ret = security_locked_down(LOCKDOWN_PCI_ACCESS);

        if (ret)
            return ret;
    }
#endif

    return 0;
}

/**
 * Copies a fixed-width binary value into a sysfs page buffer.
 *
 * Every binary show() in this driver used a bare memcpy() with no regard for
 * the size of the caller's buffer. sysfs hands show() exactly one page, so
 * the copy has to be bounded.
 */
static u32 ryzen_smu_load_u32(const char *buff)
{
    u32 v;

    /*
     * memcpy() rather than a *(u32 *) cast: the cast is an unaligned access
     * as far as the C standard is concerned and trips UBSAN's alignment
     * checker. The compiler folds this into a single load on x86.
     */
    memcpy(&v, buff, sizeof(v));
    return v;
}

static ssize_t ryzen_smu_emit_raw(char *buff, const void *src, size_t len)
{
    if (len > PAGE_SIZE)
        len = PAGE_SIZE;

    memcpy(buff, src, len);
    return len;
}

static ssize_t attr_store_null(struct kobject *kobj, struct kobj_attribute *attr, const char *buff, size_t count) {
    return 0;
}

static ssize_t drv_version_show(struct kobject *kobj, struct kobj_attribute *attr, char *buff) {
    return sysfs_emit(buff, "%s\n", THIS_MODULE->version);
}

static ssize_t version_show(struct kobject *kobj, struct kobj_attribute *attr, char *buff) {
    ssize_t ret;

    mutex_lock(&g_driver.lock);
    ret = sysfs_emit(buff, "%s\n", g_driver.smu_version);
    mutex_unlock(&g_driver.lock);

    return ret;
}

static ssize_t mp1_if_version_show(struct kobject *kobj, struct kobj_attribute *attr, char *buff) {
    return sysfs_emit(buff, "%d\n", smu_get_mp1_if_version());
}

static ssize_t codename_show(struct kobject *kobj, struct kobj_attribute *attr, char *buff) {
    return sysfs_emit(buff, "%02d\n", smu_get_codename());
}

/**
 * PM table read.
 *
 * This is a binary attribute, not a regular one, because the largest PM
 * tables are bigger than a page (Storm Peak: 0x1E48, Vermeer: 0x1BB0,
 * Matisse: 0x18AC). The original code did:
 *
 *     memcpy(buff, g_driver.pm_table, g_driver.pm_table_read_size);
 *     return g_driver.pm_table_read_size;
 *
 * into the single PAGE_SIZE buffer sysfs supplies to show(), which is a
 * straight kernel heap overflow on any of those parts.
 *
 * The refresh is done once, when userspace reads from offset 0, so a partial
 * read sequence returns a consistent snapshot rather than a mix of two
 * different sampling intervals.
 */
static ssize_t pm_table_read(struct file *filp, struct kobject *kobj,
                             RSMU_BIN_ATTR_CONST struct bin_attribute *attr,
                             char *buff, loff_t off, size_t count)
{
    ssize_t ret;

    if (off < 0)
        return -EINVAL;

    mutex_lock(&g_driver.lock);

    if (!g_driver.pm_table_enabled || !g_driver.pm_table) {
        ret = -ENODEV;
        goto out;
    }

    if (off == 0) {
        /*
         * Re-arm the requested length on every refresh. The original code
         * kept a single persistent pm_table_read_size that
         * smu_read_pm_table() overwrites with the *required* size on
         * SMU_Return_InsufficientSize. On a part whose table is larger than
         * PM_TABLE_MAX_SIZE the first read failed and left read_size above
         * the allocated buffer; the second read then passed the size check
         * and overflowed the allocation.
         */
        g_driver.pm_table_read_size = PM_TABLE_MAX_SIZE;

        if (smu_read_pm_table(g_driver.device, g_driver.pm_table,
                              &g_driver.pm_table_read_size) != SMU_Return_OK) {
            g_driver.pm_table_read_size = smu_get_pm_table_size();
            if (g_driver.pm_table_read_size > PM_TABLE_MAX_SIZE)
                g_driver.pm_table_read_size = PM_TABLE_MAX_SIZE;

            ret = -EIO;
            goto out;
        }
    }

    if (WARN_ON_ONCE(g_driver.pm_table_read_size > PM_TABLE_MAX_SIZE)) {
        ret = -EIO;
        goto out;
    }

    if ((size_t)off >= g_driver.pm_table_read_size) {
        ret = 0;
        goto out;
    }

    if (count > g_driver.pm_table_read_size - (size_t)off)
        count = g_driver.pm_table_read_size - (size_t)off;

    memcpy(buff, g_driver.pm_table + off, count);
    ret = count;

out:
    mutex_unlock(&g_driver.lock);
    return ret;
}

static struct bin_attribute bin_attr_pm_table = {
    .attr = {
        .name = "pm_table",
        .mode = RSMU_MODE_RO_HW,
    },
    .size = PM_TABLE_MAX_SIZE,
    .read = pm_table_read,
};

static ssize_t pm_table_version_show(struct kobject *kobj, struct kobj_attribute *attr, char *buff) {
    ssize_t ret;

    mutex_lock(&g_driver.lock);
    ret = ryzen_smu_emit_raw(buff, &g_driver.pm_table_version,
                             sizeof(g_driver.pm_table_version));
    mutex_unlock(&g_driver.lock);

    return ret;
}

static ssize_t pm_table_size_show(struct kobject *kobj, struct kobj_attribute *attr, char *buff) {
    ssize_t ret;

    mutex_lock(&g_driver.lock);
    ret = ryzen_smu_emit_raw(buff, &g_driver.pm_table_read_size,
                             sizeof(g_driver.pm_table_read_size));
    mutex_unlock(&g_driver.lock);

    return ret;
}

/**
 * Decodes a mailbox command ID from a sysfs write.
 *
 * Returns 0 on success. Short/oversized writes are rejected with -EINVAL
 * rather than the original "return 0", which sysfs reports as a zero-length
 * write and which turns into a spin in some writers.
 */
static int ryzen_smu_decode_op(const char *buff, size_t count, u32 *op)
{
    // To date, there has never been a command that actually exceeds FFh
    //  so 32 bits is overkill but still support it.
    switch (count) {
        case sizeof(u32):
            *op = ryzen_smu_load_u32(buff);
            return 0;
        case sizeof(u8):
            *op = *(const u8*)buff;
            return 0;
        default:
            return -EINVAL;
    }
}

static ssize_t ryzen_smu_cmd_store(const char *buff, size_t count,
                                   enum smu_mailbox mailbox)
{
    u32 op;
    int ret;

    ret = ryzen_smu_check_privileged();
    if (ret)
        return ret;

    ret = ryzen_smu_decode_op(buff, count, &op);
    if (ret)
        return ret;

    mutex_lock(&g_driver.lock);
    g_driver.smu_rsp = smu_send_command(g_driver.device, op, &g_driver.smu_args,
                                        mailbox);
    mutex_unlock(&g_driver.lock);

    return count;
}

static ssize_t smu_rsp_show(char *buff)
{
    ssize_t ret;

    mutex_lock(&g_driver.lock);
    ret = ryzen_smu_emit_raw(buff, &g_driver.smu_rsp, sizeof(g_driver.smu_rsp));
    mutex_unlock(&g_driver.lock);

    return ret;
}

static ssize_t rsmu_cmd_show(struct kobject *kobj, struct kobj_attribute *attr, char *buff) {
    return smu_rsp_show(buff);
}

static ssize_t rsmu_cmd_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buff, size_t count) {
    return ryzen_smu_cmd_store(buff, count, MAILBOX_TYPE_RSMU);
}

static ssize_t mp1_smu_cmd_show(struct kobject *kobj, struct kobj_attribute *attr, char *buff) {
    return smu_rsp_show(buff);
}

static ssize_t mp1_smu_cmd_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buff, size_t count) {
    return ryzen_smu_cmd_store(buff, count, MAILBOX_TYPE_MP1);
}

static ssize_t hsmp_smu_cmd_show(struct kobject* kobj, struct kobj_attribute* attr, char* buff) {
    return smu_rsp_show(buff);
}

static ssize_t hsmp_smu_cmd_store(struct kobject* kobj, struct kobj_attribute* attr, const char* buff, size_t count) {
    return ryzen_smu_cmd_store(buff, count, MAILBOX_TYPE_HSMP);
}

static ssize_t smu_args_show(struct kobject *kobj, struct kobj_attribute *attr, char *buff) {
    ssize_t ret;

    mutex_lock(&g_driver.lock);
    ret = ryzen_smu_emit_raw(buff, g_driver.smu_args.args,
                             sizeof(g_driver.smu_args.args));
    mutex_unlock(&g_driver.lock);

    return ret;
}

static ssize_t smu_args_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buff, size_t count) {
    int ret;

    ret = ryzen_smu_check_privileged();
    if (ret)
        return ret;

    if (count != sizeof(u32) * SMU_REQ_MAX_ARGS)
        return -EINVAL;

    mutex_lock(&g_driver.lock);
    memcpy(g_driver.smu_args.args, buff, count);
    mutex_unlock(&g_driver.lock);

    return count;
}

static ssize_t smn_show(struct kobject *kobj, struct kobj_attribute *attr, char *buff) {
    ssize_t ret;

    mutex_lock(&g_driver.lock);
    ret = ryzen_smu_emit_raw(buff, &g_driver.smn_result,
                             sizeof(g_driver.smn_result));
    mutex_unlock(&g_driver.lock);

    return ret;
}

static ssize_t smn_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buff,
size_t count) {
    u32 address, value;
    int ret;

    ret = ryzen_smu_check_privileged();
    if (ret)
        return ret;

    if (count != sizeof(u32) && count != sizeof(u32) * 2)
        return -EINVAL;

    mutex_lock(&g_driver.lock);

    if (count == sizeof(u32)) {
        // One word written means we read this address at buff[0]
        address = ryzen_smu_load_u32(buff);

        if (smu_read_address(g_driver.device, address, &g_driver.smn_result) != SMU_Return_OK) {
            pr_debug("Failed to read SMN address 0x%x\n", address);
            /*
             * The original left the previous successful result in place on
             * failure, so a caller could not tell a stale value from a fresh
             * one. Report the error instead.
             */
            g_driver.smn_result = SMU_Return_PCIFailed;
        }
    }
    else {
        // Two words written means we write the second word to the address of the first word
        address = ryzen_smu_load_u32(buff);
        value = ryzen_smu_load_u32(buff + sizeof(u32));

        if (smu_write_address(g_driver.device, address, value) != SMU_Return_OK) {
            pr_debug("Failed to write SMN address 0x%x with value 0x%x\n", address, value);
            g_driver.smn_result = SMU_Return_PCIFailed;
        }
        else
            g_driver.smn_result = SMU_Return_OK;
    }

    mutex_unlock(&g_driver.lock);

    return count;
}

__RO_ATTR (drv_version);
__RO_ATTR (version);
__RO_ATTR (mp1_if_version);
__RO_ATTR (codename);

__RO_ATTR (pm_table_size);
__RO_ATTR (pm_table_version);

__RW_ATTR (rsmu_cmd);
__RW_ATTR (mp1_smu_cmd);
__RW_ATTR (hsmp_smu_cmd);
__RW_ATTR (smu_args);

__RW_ATTR (smn);

/*
 * Optional entries are appended at probe time.
 *
 * The original code indexed backwards from MAX_ATTRS_LEN with hand-written
 * "-5 / -4 / -3 / -2" offsets and a comment warning not to touch the array.
 * An explicit cursor means adding an attribute cannot silently overwrite the
 * NULL terminator.
 */
static struct attribute *drv_attrs[MAX_ATTRS_LEN] = {
    &dev_attr_drv_version.attr,
    &dev_attr_version.attr,
    &dev_attr_mp1_if_version.attr,
    &dev_attr_codename.attr,

    &dev_attr_smu_args.attr,
    &dev_attr_mp1_smu_cmd.attr,
    &dev_attr_hsmp_smu_cmd.attr,

    &dev_attr_smn.attr,

    NULL,
};

#define DRV_ATTRS_FIXED 8
static unsigned int drv_attrs_used = DRV_ATTRS_FIXED;

static void ryzen_smu_add_attr(struct attribute *attr)
{
    /* Keep one slot for the NULL terminator. */
    if (WARN_ON_ONCE(drv_attrs_used >= MAX_ATTRS_LEN - 1))
        return;

    drv_attrs[drv_attrs_used++] = attr;
    drv_attrs[drv_attrs_used] = NULL;
}

static struct attribute_group drv_attr_group = {
    .attrs = drv_attrs,
};

static int ryzen_smu_get_version(enum smu_mailbox mb, int show) {
    u32 ver;

    ver = smu_get_version(g_driver.device, mb);

    /*
     * N.B. ver is unsigned, so the original "ver >= 0 && ver <= 0xFF" had a
     * tautological half. Values in that range are smu_return_val error codes
     * rather than a firmware version.
     */
    if (ver <= 0xFF) {
        pr_err("Failed to query the %sSMU version: %u",
            mb == MAILBOX_TYPE_RSMU ? "R" : "MP1 ", ver);
        return -EINVAL;
    }

    // In case this just tests for mailbox functionality, we don't need to output anything.
    if (show) {
        if (ver & 0xFF000000)
            snprintf(g_driver.smu_version, sizeof(g_driver.smu_version), "%d.%d.%d.%d",
                (ver >> 24) & 0xff, (ver >> 16) & 0xff, (ver >> 8) & 0xff, ver & 0xff);
        else
            snprintf(g_driver.smu_version, sizeof(g_driver.smu_version), "%d.%d.%d",
                (ver >> 16) & 0xff, (ver >> 8) & 0xff, ver & 0xff);

        pr_info("SMU v%s", g_driver.smu_version);
    }

    return 0;
}

static void ryzen_smu_teardown(void)
{
    if (g_driver.pm_table_bin_registered) {
        sysfs_remove_bin_file(g_driver.drv_kobj, &bin_attr_pm_table);
        g_driver.pm_table_bin_registered = false;
    }

    if (g_driver.sysfs_registered) {
        sysfs_remove_group(g_driver.drv_kobj, &drv_attr_group);
        g_driver.sysfs_registered = false;
    }

    if (g_driver.drv_kobj) {
        kobject_put(g_driver.drv_kobj);
        g_driver.drv_kobj = NULL;
    }

    kfree(g_driver.pm_table);
    g_driver.pm_table = NULL;
    g_driver.pm_table_enabled = false;
    g_driver.pm_table_read_size = PM_TABLE_MAX_SIZE;

    /* Reset the optional-attribute cursor so a re-probe starts clean. */
    drv_attrs_used = DRV_ATTRS_FIXED;
    drv_attrs[DRV_ATTRS_FIXED] = NULL;

    smu_cleanup();

    g_driver.device = NULL;
}

static int ryzen_smu_probe(struct pci_dev *dev, const struct pci_device_id *id) {
    enum smu_return_val ret;
    int err;

    /*
     * The driver keeps all of its state in a single global and registers a
     * fixed sysfs kobject name. Several entries in the ID table can match on
     * the same machine (and multi-socket boxes expose one root complex per
     * socket), in which case the original code would re-enter probe, leak the
     * previous PM table allocation and fail to create a duplicate kobject
     * while still returning success.
     */
    if (g_probed) {
        pr_info("Already bound to a root complex; ignoring %s", pci_name(dev));
        return -EBUSY;
    }

    mutex_init(&g_driver.lock);
    g_driver.device = dev;
    g_driver.pm_table_read_size = PM_TABLE_MAX_SIZE;

    pr_info("loading version: %s", THIS_MODULE->version);

    // Clamp values.
    if (smu_timeout_attempts > SMU_RETRIES_MAX)
        smu_timeout_attempts = SMU_RETRIES_MAX;
    if (smu_timeout_attempts < SMU_RETRIES_MIN)
        smu_timeout_attempts = SMU_RETRIES_MIN;

    // Detect processor class & figure out MP1/RSMU support.
    if (smu_init(g_driver.device) != 0) {
        pr_err("Failed to initialize the SMU for use");
        g_driver.device = NULL;
        return -ENODEV;
    }

    // Check if MP1 is working as we guarantee this support.
    if (ryzen_smu_get_version(MAILBOX_TYPE_MP1, 1) != 0) {
        pr_err("Failed to obtain the SMU version");
        err = -EINVAL;
        goto err_cleanup;
    }

    // Check if RSMU is valid to determine if to skip PM table setup.
    if (ryzen_smu_get_version(MAILBOX_TYPE_RSMU, 0) == 0) {
        ryzen_smu_add_attr(&dev_attr_rsmu_cmd.attr);
    }
    else {
        pr_info("RSMU Mailbox: Disabled or not responding to commands.");
        goto _CONTINUE_SETUP;
    }

    // Check that PM table options are supported before adding it to the attr list
    ret = smu_transfer_table_to_dram(g_driver.device);
    if (ret == SMU_Return_OK) {
        ret = smu_get_pm_table_version(g_driver.device, &g_driver.pm_table_version);
        if (ret != SMU_Return_OK && ret != SMU_Return_Unsupported) {
            pr_err("Unable to resolve which PM table version the system uses -- disabling "
                "feature (%d)", ret);
            goto _CONTINUE_SETUP;
        }

        g_driver.pm_table = kzalloc(PM_TABLE_MAX_SIZE, GFP_KERNEL);
        if (g_driver.pm_table == NULL) {
            pr_err("Unable to allocate kernel buffer for PM table mapping -- disabling PM table "
                "feature");
            goto _CONTINUE_SETUP;
        }

        // Perform an initial fill of the data for when the device is queued, saving time
        pr_debug("Probing the PM table for state changes");
        g_driver.pm_table_read_size = PM_TABLE_MAX_SIZE;
        ret = smu_read_pm_table(dev, g_driver.pm_table, &g_driver.pm_table_read_size);
        if (ret == SMU_Return_OK) {
            pr_debug("Probe succeeded: read %zu bytes", g_driver.pm_table_read_size);

            g_driver.pm_table_enabled = true;

            ryzen_smu_add_attr(&dev_attr_pm_table_size.attr);

            if (g_driver.pm_table_version)
                ryzen_smu_add_attr(&dev_attr_pm_table_version.attr);
        }
        else {
            pr_err("Failed to probe the PM table -- disabling feature (%d)", ret);

            kfree(g_driver.pm_table);
            g_driver.pm_table = NULL;
            g_driver.pm_table_read_size = PM_TABLE_MAX_SIZE;
        }
    }
    else {
        pr_debug("Notice: PM tables are not supported for the current platform (%d)", ret);
    }

_CONTINUE_SETUP:
    // Allocate the sysfs attr group with the parameters for use
    g_driver.drv_kobj = kobject_create_and_add("ryzen_smu_drv", kernel_kobj);
    if (!g_driver.drv_kobj) {
        pr_err("Unable to create sysfs interface");
        err = -ENOMEM;
        goto err_cleanup;
    }

    /*
     * The original code called kobject_put() on failure here but still
     * returned 0, leaving a freed kobject in g_driver.drv_kobj that
     * ryzen_smu_remove() would put a second time.
     */
    err = sysfs_create_group(g_driver.drv_kobj, &drv_attr_group);
    if (err) {
        pr_err("Unable to register the sysfs attribute group (%d)", err);
        goto err_cleanup;
    }
    g_driver.sysfs_registered = true;

    if (g_driver.pm_table_enabled) {
        err = sysfs_create_bin_file(g_driver.drv_kobj, &bin_attr_pm_table);
        if (err) {
            pr_err("Unable to register the pm_table attribute (%d) -- disabling feature", err);
            g_driver.pm_table_enabled = false;
        }
        else
            g_driver.pm_table_bin_registered = true;
    }

    g_probed = true;
    return 0;

err_cleanup:
    ryzen_smu_teardown();
    return err;
}

static void ryzen_smu_remove(struct pci_dev *dev) {
    if (!g_probed)
        return;

    ryzen_smu_teardown();
    g_probed = false;
}

static struct pci_device_id ryzen_smu_id_table[] = {
    { PCI_DEVICE(PCI_VENDOR_ID_AMD, PCI_DEVICE_ID_AMD_17H_ROOT) },
    { PCI_DEVICE(PCI_VENDOR_ID_AMD, PCI_DEVICE_ID_AMD_17H_M10H_ROOT) },
    { PCI_DEVICE(PCI_VENDOR_ID_AMD, PCI_DEVICE_ID_AMD_17H_M30H_ROOT) },
    { PCI_DEVICE(PCI_VENDOR_ID_AMD, PCI_DEVICE_ID_AMD_17H_M60H_ROOT) },
    { PCI_DEVICE(PCI_VENDOR_ID_AMD, PCI_DEVICE_ID_AMD_1AH_M00H_ROOT) },
    { PCI_DEVICE(PCI_VENDOR_ID_AMD, PCI_DEVICE_ID_AMD_1AH_M20H_ROOT) },
    { PCI_DEVICE(PCI_VENDOR_ID_AMD, PCI_DEVICE_ID_AMD_1AH_M60H_ROOT) },
    { PCI_DEVICE(PCI_VENDOR_ID_AMD, PCI_DEVICE_ID_AMD_17H_MA0H_ROOT) },
    { PCI_DEVICE(PCI_VENDOR_ID_AMD, PCI_DEVICE_ID_AMD_19H_M10H_ROOT) },
    { PCI_DEVICE(PCI_VENDOR_ID_AMD, PCI_DEVICE_ID_AMD_19H_M40H_ROOT) },
    { PCI_DEVICE(PCI_VENDOR_ID_AMD, PCI_DEVICE_ID_AMD_19H_M60H_ROOT) },
    { PCI_DEVICE(PCI_VENDOR_ID_AMD, PCI_DEVICE_ID_AMD_19H_M70H_ROOT) },
    { PCI_DEVICE(PCI_VENDOR_ID_AMD, PCI_DEVICE_ID_AMD_MI200_ROOT) },
    { PCI_DEVICE(PCI_VENDOR_ID_AMD, PCI_DEVICE_ID_AMD_MI300_ROOT) },
    { }
};
MODULE_DEVICE_TABLE(pci, ryzen_smu_id_table);

static struct pci_driver ryzen_smu_driver = {
    .id_table = ryzen_smu_id_table,
    .remove = ryzen_smu_remove,
    .probe = ryzen_smu_probe,
    .name = KBUILD_MODNAME,
};

static int __init ryzen_smu_driver_init(void) {
    int ret;

    // By default the driver will not be used to communicate with the
    //  northbridge so we forcefully tell the system to use it.
    ret = pci_register_driver(&ryzen_smu_driver);
    if (ret < 0) {
        pr_err("Failed to register the PCI driver.");
        /* Propagate the real errno; the original returned a bare 1. */
        return ret;
    }

    return 0;
}

static void ryzen_smu_driver_exit(void) {
    pci_unregister_driver(&ryzen_smu_driver);
}

module_init(ryzen_smu_driver_init);
module_exit(ryzen_smu_driver_exit);

/**
 * The parameter stays writable so it can be tuned on a live system, but the
 * clamp now also happens at the point of use in smu_send_command(): a runtime
 * write of 0 previously underflowed the "retries--" counter into a ~4 billion
 * iteration busy loop held under the mailbox mutex.
 */
module_param(smu_timeout_attempts, uint, RSMU_MODE_RW);
MODULE_PARM_DESC(smu_timeout_attempts, "When executing an SMU command, the driver will retry this many times before considering a command to have timed out. Default: 8192");
