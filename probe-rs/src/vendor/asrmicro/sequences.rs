use crate::{
    MemoryMappedRegister, RegisterId,
    architecture::arm::{
        ArmDebugInterface, ArmError, FullyQualifiedApAddress,
        armv7m::Dhcsr,
        core::cortex_m::write_core_reg,
        memory::ArmMemoryInterface,
        sequences::{ArmDebugSequence, cortex_m_core_start},
    },
};
use probe_rs_target::CoreType;
use std::sync::Arc;
use std::time::{Duration, Instant};

#[derive(Debug)]
pub struct Asr6601;

impl Asr6601 {
    pub fn create() -> Arc<Self> {
        Arc::new(Self)
    }
}

const VECTOR_TABLE: u64 = 0x0800_0000;
const FLASH_RANGE: core::ops::RangeInclusive<u32> = 0x0800_0000..=0x0803_FFFF;
// RAM is 0x20000000..0x20010000; the initial SP is the region end (0x20010000).
const RAM_RANGE: core::ops::RangeInclusive<u32> = 0x2000_0000..=0x2001_0000;

const ICTR: u64 = 0xE000_E004;
const SYST_CSR: u64 = 0xE000_E010;
const SYST_RVR: u64 = 0xE000_E014;
const SYST_CVR: u64 = 0xE000_E018;
const NVIC_ICER: u64 = 0xE000_E180;
const NVIC_ICPR: u64 = 0xE000_E280;
const SCB_ICSR: u64 = 0xE000_ED04;
const SCB_VTOR: u64 = 0xE000_ED08;
const SCB_SCR: u64 = 0xE000_ED10;
const SCB_SHCSR: u64 = 0xE000_ED24;

// RCC base 0x4000_0000 — RM §8.3.4 RCC_CGR0 (offset 0x00C).
const RCC_CGR0: u64 = 0x4000_000C;
/// RM §8.3.4 bit 21: clock gate for the SYSCFG peripheral.
const RCC_CGR0_SYSCFG_CLK_EN: u32 = 1 << 21;

// SYSCFG base 0x4000_1000 — RM §7.5.
const SYSCFG_CR2: u64 = 0x4000_1008;
const SYSCFG_CR3: u64 = 0x4000_100C;
/// SYSCFG_CR2 bit 10: allow debug while the CPU is in Sleep/Deepsleep.
const SYSCFG_DBG_SLEEP: u32 = 1 << 10;
/// SYSCFG_CR3 bit 1: allow debug while the CPU is in Stop.
const SYSCFG_DBG_STOP: u32 = 1 << 1;
/// SYSCFG_CR3 bit 0: allow debug while the CPU is in Standby.
const SYSCFG_DBG_STANDBY: u32 = 1 << 0;

const REG_SP: RegisterId = RegisterId(13);
const REG_LR: RegisterId = RegisterId(14);
const REG_PC: RegisterId = RegisterId(15);
const REG_XPSR: RegisterId = RegisterId(16);
const REG_MSP: RegisterId = RegisterId(17);
const REG_PSP: RegisterId = RegisterId(18);
// { CONTROL[7:0], FAULTMASK[7:0], BASEPRI[7:0], PRIMASK[7:0] }
const REG_SPECIAL: RegisterId = RegisterId(20);

/// Keep SWD working after the application executes WFI/WFE.
///
/// By default the ASR6601 powers down the debug connection when entering low-power
/// modes (Sleep / Stop / Standby). Once that happens the probe times out and cannot
/// reattach until a power cycle or BOOT0 recovery.
///
/// The chip has three sticky "keep debug on" bits in SYSCFG. We set all three so any
/// low-power mode is debug-safe.
fn enable_debug_during_sleep(memory: &mut dyn ArmMemoryInterface) -> Result<(), ArmError> {
    // SYSCFG_CR2 is on the gated SYSCFG clock; we need to enable it before touching CR2.
    let cgr0 = memory.read_word_32(RCC_CGR0)?;
    if cgr0 & RCC_CGR0_SYSCFG_CLK_EN == 0 {
        memory.write_word_32(RCC_CGR0, cgr0 | RCC_CGR0_SYSCFG_CLK_EN)?;
    }

    // SYSCFG_DBG_SLEEP = 1 → "allowed" to keep a debug connection
    // in Sleep/Deepsleep (covers ordinary WFE/WFI with SLEEPDEEP = 0/1).
    let cr2 = memory.read_word_32(SYSCFG_CR2)?;
    memory.write_word_32(SYSCFG_CR2, cr2 | SYSCFG_DBG_SLEEP)?;

    // SYSCFG_DBG_STOP / SYSCFG_DBG_STANDBY = 1 → keep debug
    // when firmware later enters Stop0–3 or Standby (SLEEPDEEP + PWR lp_mode).
    let cr3 = memory.read_word_32(SYSCFG_CR3)?;
    memory.write_word_32(SYSCFG_CR3, cr3 | SYSCFG_DBG_STOP | SYSCFG_DBG_STANDBY)?;

    Ok(())
}

// Flash option bytes, RM §10.4: 64-bit Option0 selects the boot mode.
// Bit 0 FLASH_BOOT0 + bit 1 USE_FLASH_BOOT0 = 1 boots Main flash even with
// the BOOT0 pin (GPIO02) tied high (RM Table 10-3). Bits 12-5 DEBUG_LEVEL:
// 0xCC seals the debug port irreversibly, so it is never written here.
const OPT0_ADDR: u64 = 0x1000_3000;
const OPT0_BOOT_BITS_MASK: u32 = 0x3;
const OPT0_DEBUG_LEVEL_MASK: u32 = 0xFF;
const OPT0_DEBUG_LEVEL_SHIFT: u32 = 5;
const OPT0_DEBUG_LEVEL_SEALED: u32 = 0xCC;

// EFC, RM §10.5, base 0x4002_0000.
const EFC_CR: u64 = 0x4002_0000;
const EFC_SR: u64 = 0x4002_0008;
const EFC_PROG_DATA0: u64 = 0x4002_000C;
const EFC_PROG_DATA1: u64 = 0x4002_0010;
const EFC_PROTECT_SEQ: u64 = 0x4002_0018;
const EFC_OPTION_CSR: u64 = 0x4002_003C;
const PROTECT_SEQ0: u32 = 0x8C9D_AEBF;
const PROTECT_SEQ1: u32 = 0x1314_1516;
const EFC_CR_OPTION_OPR_EN: u32 = 1 << 8;
const EFC_CR_PREFETCH_EN: u32 = 1 << 5;
const EFC_CR_ECC_DIS: u32 = 1 << 9;
const EFC_CR_INFO_BYTE_LOAD: u32 = 1 << 31;
const EFC_SR_OPERATION_DONE: u32 = 1 << 0;
const EFC_SR_OPTION_WR_ERR: u32 = 1 << 4;

/// What the boot-option inspection decided.
#[derive(Debug, PartialEq, Eq)]
enum BootCheck {
    /// USE_FLASH_BOOT0 + FLASH_BOOT0 already set; nothing to do.
    Ok,
    /// Program these 8 bytes over the Option0 slot (bits 1:0 set, rest kept).
    FixOptions { opt0l: u32, opt0h: u32 },
    /// Leave the device untouched; human must intervene.
    Refuse(&'static str),
}

/// Pure decision logic: no hardware access, unit-tested below.
///
/// `raw_l` is the low word of the Option0 flash slot, `csr` the reloaded
/// OPTION_CSR mirror. Both use the same bit positions for the boot
/// bits (0: FLASH_BOOT0, 1: USE_FLASH_BOOT0, 2: FLASH_BOOT1).
fn plan_option_fix(raw_l: u32, csr: u32) -> BootCheck {
    if (raw_l >> OPT0_DEBUG_LEVEL_SHIFT) & OPT0_DEBUG_LEVEL_MASK == OPT0_DEBUG_LEVEL_SEALED {
        return BootCheck::Refuse("Option0 DEBUG_LEVEL is 0xCC (sealed); refusing to modify");
    }
    if (csr >> 5) & 0x3 == 2 {
        return BootCheck::Refuse("live DebugLevel is Level 2 (sealed); refusing to modify");
    }
    if (raw_l & 0x7) != (csr & 0x7) {
        return BootCheck::Refuse("raw Option0 boot bits disagree with CSR mirror");
    }
    if raw_l & OPT0_BOOT_BITS_MASK == OPT0_BOOT_BITS_MASK {
        BootCheck::Ok
    } else {
        // Read-modify-write is done by the caller on the full 64-bit slot;
        // only bits 1:0 change, everything else is preserved bit-exact.
        BootCheck::FixOptions {
            opt0l: raw_l | OPT0_BOOT_BITS_MASK,
            opt0h: 0, // filled in by the caller, which holds the high word
        }
    }
}

fn efc_unlock(memory: &mut dyn ArmMemoryInterface) -> Result<(), ArmError> {
    memory.write_word_32(EFC_PROTECT_SEQ, PROTECT_SEQ0)?;
    memory.write_word_32(EFC_PROTECT_SEQ, PROTECT_SEQ1)?;
    Ok(())
}

fn efc_lock(memory: &mut dyn ArmMemoryInterface) -> Result<(), ArmError> {
    memory.write_word_32(EFC_PROTECT_SEQ, PROTECT_SEQ0)?;
    memory.write_word_32(EFC_PROTECT_SEQ, 0)?;
    Ok(())
}

/// Wait for OPERATION_DONE; error out on OPTION_WR_ERR or timeout.
fn efc_wait_done(memory: &mut dyn ArmMemoryInterface) -> Result<(), ArmError> {
    let start = Instant::now();
    loop {
        let sr = memory.read_word_32(EFC_SR)?;
        if sr & EFC_SR_OPTION_WR_ERR != 0 {
            return Err(ArmError::Other(
                "ASR6601: EFC rejected the option value (OPTION_WR_ERR)".into(),
            ));
        }
        if sr & EFC_SR_OPERATION_DONE != 0 {
            // Write-1-to-clear the done flag.
            memory.write_word_32(EFC_SR, sr)?;
            return Ok(());
        }
        if start.elapsed() >= Duration::from_secs(2) {
            return Err(ArmError::Timeout);
        }
        std::thread::sleep(Duration::from_millis(1));
    }
}

/// Reload the OPTION_* live mirrors from flash info without resetting the core.
fn efc_info_reload(memory: &mut dyn ArmMemoryInterface) -> Result<(), ArmError> {
    efc_unlock(memory)?;
    let cr = memory.read_word_32(EFC_CR)?;
    memory.write_word_32(EFC_CR, cr | EFC_CR_INFO_BYTE_LOAD)?;
    efc_lock(memory)?;
    Ok(())
}

/// Program one 8-byte Option0 unit with the EFC single-program flow.
/// The core must already be halted: flash cannot execute mid-operation.
fn efc_program_option0(
    memory: &mut dyn ArmMemoryInterface,
    data0: u32,
    data1: u32,
) -> Result<(), ArmError> {
    let ecc_dis = memory.read_word_32(EFC_CR)? & EFC_CR_ECC_DIS;
    efc_unlock(memory)?;
    memory.write_word_32(EFC_CR, EFC_CR_OPTION_OPR_EN | EFC_CR_PREFETCH_EN | ecc_dis)?;
    efc_lock(memory)?;

    memory.write_word_32(EFC_PROG_DATA0, data0)?;
    memory.write_word_32(EFC_PROG_DATA1, data1)?;
    // Any store to the target address triggers the operation.
    memory.write_word_32(OPT0_ADDR, 0xFFFF_FFFF)?;
    efc_wait_done(memory)
}

/// Ensure the chip boots Main flash even with BOOT0 tied high.
/// Runs at every session start; only modifies the device when required.
fn ensure_boot_options(memory: &mut dyn ArmMemoryInterface) -> Result<(), ArmError> {
    // Never program flash while the target is in reset: EFC operations
    // during reboot risk corruption.
    if Dhcsr(memory.read_word_32(Dhcsr::get_mmio_address())?).s_reset_st() {
        tracing::debug!("ASR6601: core is in reset, skipping boot-option check");
        return Ok(());
    }

    let raw_l = memory.read_word_32(OPT0_ADDR)?;
    let raw_h = memory.read_word_32(OPT0_ADDR + 4)?;

    // Mirrors can be stale (emulated resets never reload them).
    efc_info_reload(memory)?;
    let csr = memory.read_word_32(EFC_OPTION_CSR)?;

    match plan_option_fix(raw_l, csr) {
        BootCheck::Refuse(reason) => {
            tracing::error!("ASR6601: {reason}; leaving option bytes untouched");
            return Ok(());
        }
        BootCheck::Ok => {
            tracing::debug!("ASR6601: boot options already bypass BOOT0 (USE=1 BOOT0=1)");
        }
        BootCheck::FixOptions { opt0l, .. } => {
            tracing::warn!(
                "ASR6601: programming Option0 {raw_l:#010X} -> {opt0l:#010X} \
                 (USE_FLASH_BOOT0 + FLASH_BOOT0) so Main flash boots with BOOT0 high"
            );
            efc_program_option0(memory, opt0l, raw_h)?;
            let vrf_l = memory.read_word_32(OPT0_ADDR)?;
            let vrf_h = memory.read_word_32(OPT0_ADDR + 4)?;
            if vrf_l != opt0l || vrf_h != raw_h {
                return Err(ArmError::Other(
                    "ASR6601: Option0 verify mismatch after programming".into(),
                ));
            }
            efc_info_reload(memory)?;
            let csr2 = memory.read_word_32(EFC_OPTION_CSR)?;
            if csr2 & OPT0_BOOT_BITS_MASK != OPT0_BOOT_BITS_MASK {
                return Err(ArmError::Other(
                    "ASR6601: reloaded CSR mirrors do not show the new boot bits".into(),
                ));
            }
            tracing::warn!(
                "ASR6601: boot options programmed and verified; take effect at next real reset"
            );
        }
    }

    Ok(())
}

fn halt_core(memory: &mut dyn ArmMemoryInterface) -> Result<(), ArmError> {
    let start = Instant::now();
    loop {
        let mut dhcsr = Dhcsr(0);
        dhcsr.set_c_debugen(true);
        dhcsr.set_c_halt(true);
        dhcsr.enable_write();
        memory.write_word_32(Dhcsr::get_mmio_address(), dhcsr.into())?;

        if Dhcsr(memory.read_word_32(Dhcsr::get_mmio_address())?).s_halt() {
            return Ok(());
        }
        if start.elapsed() >= Duration::from_millis(500) {
            return Err(ArmError::Timeout);
        }
        std::thread::sleep(Duration::from_millis(5));
    }
}

impl ArmDebugSequence for Asr6601 {
    /// SYSRESETREQ drops the ASR6601 debug connection and does not reliably boot
    /// the application under reset catch. Emulate the architectural reset state
    /// while preserving the debug connection.
    fn reset_system(
        &self,
        interface: &mut dyn ArmMemoryInterface,
        _core_type: CoreType,
        _debug_base: Option<u64>,
    ) -> Result<(), ArmError> {
        let start = Instant::now();
        loop {
            let mut dhcsr = Dhcsr(0);
            dhcsr.set_c_debugen(true);
            dhcsr.set_c_halt(true);
            dhcsr.enable_write();
            interface.write_word_32(Dhcsr::get_mmio_address(), dhcsr.into())?;

            if Dhcsr(interface.read_word_32(Dhcsr::get_mmio_address())?).s_halt() {
                break;
            }
            if start.elapsed() >= Duration::from_millis(500) {
                return Err(ArmError::Timeout);
            }
            std::thread::sleep(Duration::from_millis(5));
        }

        let sp = interface.read_word_32(VECTOR_TABLE)?;
        let pc = interface.read_word_32(VECTOR_TABLE + 4)?;
        if !RAM_RANGE.contains(&sp) || !FLASH_RANGE.contains(&(pc & !1)) || pc & 1 == 0 {
            tracing::warn!(
                "vector table at {VECTOR_TABLE:#010x} is invalid (SP={sp:#010x}, PC={pc:#010x}), \
                 continuing with reset anyway"
            );
        }

        interface.write_word_32(SYST_CSR, 0)?;
        interface.write_word_32(SYST_RVR, 0)?;
        interface.write_word_32(SYST_CVR, 0)?;

        let interrupt_registers = (interface.read_word_32(ICTR)? & 0x0f) + 1;
        for index in 0..interrupt_registers {
            let offset = u64::from(index) * 4;
            interface.write_word_32(NVIC_ICER + offset, u32::MAX)?;
            interface.write_word_32(NVIC_ICPR + offset, u32::MAX)?;
        }

        interface.write_word_32(SCB_ICSR, (1 << 25) | (1 << 27))?;
        interface.write_word_32(SCB_SCR, 0)?;
        interface.write_word_32(SCB_SHCSR, 0)?;
        interface.write_word_32(SCB_VTOR, VECTOR_TABLE as u32)?;

        write_core_reg(interface, REG_SPECIAL, 0)?;
        write_core_reg(interface, REG_XPSR, 1 << 24)?;
        write_core_reg(interface, REG_PSP, 0)?;
        write_core_reg(interface, REG_MSP, sp)?;
        write_core_reg(interface, REG_SP, sp)?;
        write_core_reg(interface, REG_LR, u32::MAX)?;
        write_core_reg(interface, REG_PC, pc)?;

        enable_debug_during_sleep(interface)?;

        Ok(())
    }

    /// Session start: default core init, halt, then make sure the chip can
    /// boot Main flash with BOOT0 tied high. Only the Option0 boot bits are
    /// touched; everything else is left alone.
    fn debug_core_start(
        &self,
        interface: &mut dyn ArmDebugInterface,
        core_ap: &FullyQualifiedApAddress,
        core_type: CoreType,
        _debug_base: Option<u64>,
        _cti_base: Option<u64>,
    ) -> Result<(), ArmError> {
        let mut core = interface.memory_interface(core_ap)?;

        match core_type {
            CoreType::Armv6m | CoreType::Armv7m | CoreType::Armv7em | CoreType::Armv8m => {
                cortex_m_core_start(&mut *core)
            }
            _ => {
                return Err(ArmError::Other("ASR6601: unexpected core type".into()));
            }
        }?;

        // The EFC program sequence forbids flash execution mid-operation.
        halt_core(&mut *core)?;
        ensure_boot_options(&mut *core)?;

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // CSR layout mirrors Option0 boot bits: 0 FLASH_BOOT0, 1 USE, 2 BOOT1.
    const CSR_USE_BOOT: u32 = 0x7;
    const CSR_PIN_BOOTLOADER: u32 = 0x5; // BOOT0=1 USE=0 BOOT1=1

    #[test]
    fn already_fixed_needs_nothing() {
        assert_eq!(plan_option_fix(0xFC07_F557, CSR_USE_BOOT), BootCheck::Ok);
    }

    #[test]
    fn pin_bootloader_gets_fixed() {
        // Factory state on BOOT0-to-VDD boards: BOOT0=1 USE=0 BOOT1=1.
        assert_eq!(
            plan_option_fix(0xFC07_F555, CSR_PIN_BOOTLOADER),
            BootCheck::FixOptions {
                opt0l: 0xFC07_F557,
                opt0h: 0,
            }
        );
    }

    #[test]
    fn sealed_debug_level_refuses() {
        // DEBUG_LEVEL field 0xCC in raw Option0.
        assert_eq!(
            plan_option_fix(0xFC07_F995, CSR_PIN_BOOTLOADER),
            BootCheck::Refuse("Option0 DEBUG_LEVEL is 0xCC (sealed); refusing to modify")
        );
        // Live Level 2 in CSR.
        assert_eq!(
            plan_option_fix(0xFC07_F555, CSR_PIN_BOOTLOADER | (2 << 5)),
            BootCheck::Refuse("live DebugLevel is Level 2 (sealed); refusing to modify")
        );
    }

    #[test]
    fn mirror_disagreement_refuses() {
        assert_eq!(
            plan_option_fix(0xFC07_F557, CSR_PIN_BOOTLOADER),
            BootCheck::Refuse("raw Option0 boot bits disagree with CSR mirror")
        );
    }
}
