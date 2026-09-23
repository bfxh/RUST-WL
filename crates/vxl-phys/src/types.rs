//! types：从 lib.rs 按域拆出（纯搬移，语义未改）。

/// 提供者条目（统一 id 空间：体素体 / 高斯喷溅场…）。
pub enum ProviderEntry {
    Voxel(vxl_phys_terrain::voxel::VoxelVolume),
    /// **高斯喷溅场**（喷溅域的物理代理：隐式场提供者，见 `vxl-phys-splat`）。
    Splat(vxl_phys_splat::GaussianSplatField),
    /// **三角网格**（网格域：静态关卡几何，薄壳接触，见 `vxl-phys-terrain::mesh`）。
    Mesh(vxl_phys_terrain::mesh::TriMesh),
}
