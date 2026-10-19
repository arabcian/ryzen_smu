# ryzen_smu 0.1.9 — RyzenAdj ile birlikte kullanım

- **Yeni `smu_raw_cmd` (0600, root):** tek bir atomik SMU mailbox işlemi.
  Yaz: 10×u32 `{cmd_addr, rsp_addr, args_addr, msg_id, arg0..arg5}`; oku: 7×u32 `{status, arg0..arg5}`.
  İşlem sürücünün kendi `amd_smu_mutex`'i altında çalışır; RyzenAdj artık mailbox'ı `smn`
  üzerinden register register sürüp sürücünün PM table / rsmu_cmd / mp1_smu_cmd işlemleriyle
  çakışmaz. Sonuç yazan sürece (tgid) özeldir: başka süreç okursa `-EAGAIN`, yazmadan okursa
  `-ENODATA`. Adresler MP1/SMU C2PMSG penceresiyle (0x03B10000–0x03B10FFF, 4-hizalı) sınırlı.
- `smu_send_command()` → `smu_send_command_at()` (adres tabanlı) + mailbox seçici sarmalayıcı.
- `asm/cpuid/api.h` yalnızca yeni kernellerde var; `__has_include` ile eski kernellerde
  `asm/processor.h`'e düşülüyor (önceden 6.8'de derleme hatası).
- `MAX_ATTRS_LEN` 12→14 (yeni sabit attribute için yer).
- Sürüm 0.1.9 (RyzenAdj'ın `0.1.x, x>=7` kontrolüyle uyumlu).

## Donanım güvenliği düzeltmeleri (0.1.9 üzerine)

- **`smu_args` + `*_cmd` süreçler arası karışma:** argümanlar ve yanıt, `smu_args`'ı son yazan sürece (tgid) ait.
  Başka süreç araya argüman yazdıysa `*_cmd` yazımı `-EBUSY`, `smu_args`/`*_cmd` okuması `-EAGAIN` döner.
  Önceden A sürecinin komutu B'nin argümanlarıyla (ör. başka bir güç/akım limiti) SMU'ya gidebiliyordu.
- **`smn` yazımı C2PMSG penceresine (0x03B10000–0x03B10FFF) düşüyorsa `amd_smu_mutex` altında yapılıyor;**
  sürücünün kendi mailbox işleminin ortasına RSP/ARG/CMD yazılamıyor.
- **`smu_raw_cmd`:** cmd / rsp / args aralıkları çakışamaz (`-EINVAL`). Çakışma, argümanın CMD'ye yazılıp
  rastgele mesaj tetiklemesine ya da RSP'ye yazılıp meşgul SMU'ya yeni komut gönderilmesine yol açıyordu.
- **Mailbox yarım programlanırsa** (RSP sıfırlandı, CMD yazılamadı) RSP eski boşta değerine geri yazılıyor;
  önceden mailbox reboot'a kadar kalıcı timeout'a düşüyordu.
