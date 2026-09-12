//! # vxl-phys-ffi
//!
//! C ABI 批量接口（§9，仅编辑器/外部工具）—— M2 前 keep-out，本 crate 固定 ABI 约定：
//! - **批量**接口（整世界快照/指令块进出），禁止每对象每帧跨界；
//! - 结构体带 `size: u32, version: u32` 头；
//! - 错误一律错误码；异常/panic 不跨界（panic = unwind catch 转码）；
//! - 序列化：二进制版本化（schema 版本字段 + 拒绝不兼容），零拷贝读路径
//!   （bytemuck 校验后，M2 引入依赖）。

#![forbid(unsafe_code)]

/// ABI 契约版本（不兼容变更必须递增）。
pub const ABI_VERSION: u32 = 1;

/// 所有跨界结构体的公共头。
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct BlockHeader {
    pub size: u32,
    pub version: u32,
}

/// 错误码（不跨界 panic/异常）。
#[repr(u32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FfiError {
    Ok = 0,
    NullPointer = 1,
    VersionMismatch = 2,
    SizeMismatch = 3,
    NotReady = 4,
    PanicCaught = 5,
}

/// 引擎句柄（M2 实现；当前恒返回 NotReady）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EngineHandle;

/// 创建引擎句柄（约定：C 侧传入期望 ABI 版本）。
pub fn vxl_engine_create(expected_version: u32) -> Result<EngineHandle, FfiError> {
    if expected_version != ABI_VERSION {
        return Err(FfiError::VersionMismatch);
    }
    Err(FfiError::NotReady)
}

/// 整世界快照导出（批量；M2 实现）。
pub fn vxl_engine_export_snapshot(_h: &EngineHandle, _out: &mut [u8]) -> FfiError {
    FfiError::NotReady
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn version_mismatch_rejected() {
        assert_eq!(vxl_engine_create(ABI_VERSION), Err(FfiError::NotReady));
        assert_eq!(
            vxl_engine_create(ABI_VERSION + 1),
            Err(FfiError::VersionMismatch)
        );
    }

    #[test]
    fn header_layout() {
        let h = BlockHeader {
            size: 8,
            version: ABI_VERSION,
        };
        assert_eq!(h.version, 1);
    }
}
