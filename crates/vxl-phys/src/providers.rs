//! providers：从 lib.rs 按域拆出（纯搬移，语义未改）。
use super::*;

/// 外部碰撞提供者集合（门面持有；实现 `interop::ProviderColliders` 供窄相查询）。
#[derive(Default)]
pub struct Providers {
    pub(crate) entries: Vec<ProviderEntry>,
}

impl Providers {
    /// 注册体素体，返回其 id（= 注册序，全提供者共用一个 id 空间）。
    pub fn push(&mut self, vol: vxl_phys_terrain::voxel::VoxelVolume) -> u32 {
        let id = self.entries.len() as u32;
        self.entries.push(ProviderEntry::Voxel(vol));
        id
    }

    /// 注册高斯喷溅场（同 id 空间）。
    pub fn push_splat(&mut self, field: vxl_phys_splat::GaussianSplatField) -> u32 {
        let id = self.entries.len() as u32;
        self.entries.push(ProviderEntry::Splat(field));
        id
    }

    /// 注册三角网格（静态关卡；同 id 空间）。
    pub fn push_mesh(&mut self, mesh: vxl_phys_terrain::mesh::TriMesh) -> u32 {
        let id = self.entries.len() as u32;
        self.entries.push(ProviderEntry::Mesh(mesh));
        id
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// provider(id) 的世界包围盒（宽相 AABB 供给；与 `CollisionProvider::bounds` 同义）。
    pub fn bounds(&self, id: u32) -> Option<Aabb> {
        use vxl_phys_core::interop::CollisionProvider;
        match self.entries.get(id as usize)? {
            ProviderEntry::Voxel(v) => Some(v.bounds()),
            ProviderEntry::Splat(f) => {
                use vxl_phys_core::interop::ProviderColliders;
                f.bounds(id)
            }
            ProviderEntry::Mesh(m) => {
                use vxl_phys_core::interop::ProviderColliders;
                m.bounds(id)
            }
        }
    }

    pub fn voxel(&self, id: u32) -> Option<&vxl_phys_terrain::voxel::VoxelVolume> {
        match self.entries.get(id as usize)? {
            ProviderEntry::Voxel(v) => Some(v),
            _ => None,
        }
    }

    /// 网格只读视图（渲染/诊断）。
    pub fn mesh(&self, id: u32) -> Option<&vxl_phys_terrain::mesh::TriMesh> {
        match self.entries.get(id as usize)? {
            ProviderEntry::Mesh(m) => Some(m),
            _ => None,
        }
    }

    pub fn voxel_mut(&mut self, id: u32) -> Option<&mut vxl_phys_terrain::voxel::VoxelVolume> {
        match self.entries.get_mut(id as usize)? {
            ProviderEntry::Voxel(v) => Some(v),
            _ => None,
        }
    }

    /// 喷溅场只读视图（渲染桥/诊断）。
    pub fn splat(&self, id: u32) -> Option<&vxl_phys_splat::GaussianSplatField> {
        match self.entries.get(id as usize)? {
            ProviderEntry::Splat(f) => Some(f),
            _ => None,
        }
    }
}

impl vxl_phys_core::interop::ProviderColliders for Providers {
    fn bounds(&self, id: u32) -> Option<Aabb> {
        use vxl_phys_core::interop::CollisionProvider;
        match self.entries.get(id as usize)? {
            ProviderEntry::Voxel(v) => Some(v.bounds()),
            ProviderEntry::Splat(f) => f.bounds(id),
            ProviderEntry::Mesh(m) => m.bounds(id),
        }
    }

    fn contacts_box(
        &self,
        id: u32,
        half: Vec3,
        pos: Vec3,
        rot: Quat,
        skin: f32,
        out: &mut Vec<vxl_phys_core::interop::InteropContact>,
    ) -> bool {
        match self.entries.get(id as usize) {
            Some(ProviderEntry::Voxel(v)) => {
                vxl_phys_terrain::voxel::contacts_box_voxel(v, half, pos, rot, skin, out)
            }
            Some(ProviderEntry::Splat(f)) => f.contacts_box(id, half, pos, rot, skin, out),
            Some(ProviderEntry::Mesh(m)) => m.contacts_box(id, half, pos, rot, skin, out),
            None => false,
        }
    }

    fn contacts_point(
        &self,
        id: u32,
        p: Vec3,
        skin: f32,
        out: &mut Vec<vxl_phys_core::interop::InteropContact>,
    ) -> bool {
        match self.entries.get(id as usize) {
            Some(ProviderEntry::Voxel(v)) => {
                vxl_phys_terrain::voxel::contacts_point_voxel(v, p, skin, out)
            }
            Some(ProviderEntry::Splat(f)) => f.contacts_point(id, p, skin, out),
            Some(ProviderEntry::Mesh(m)) => m.contacts_point(id, p, skin, out),
            None => false,
        }
    }

    /// 流体边界口径：体素走内点鲁棒变体（截断 SDF 在薄壁内部被格间内面
    /// 主导 ⇒ 中心差分法线可指向固体深处，投影穿壁隧逃——见切片1实测）；
    /// 其余提供者（解析面/半空间无内点歧义）沿用 `contacts_point`。
    fn contacts_point_boundary(
        &self,
        id: u32,
        p: Vec3,
        skin: f32,
        out: &mut Vec<vxl_phys_core::interop::InteropContact>,
    ) -> bool {
        match self.entries.get(id as usize) {
            Some(ProviderEntry::Voxel(v)) => {
                vxl_phys_terrain::voxel::contacts_point_voxel_solid(v, p, skin, out)
            }
            _ => self.contacts_point(id, p, skin, out),
        }
    }

    fn contacts_sphere(
        &self,
        id: u32,
        center: Vec3,
        radius: f32,
        skin: f32,
        out: &mut Vec<vxl_phys_core::interop::InteropContact>,
    ) -> bool {
        match self.entries.get(id as usize) {
            Some(ProviderEntry::Voxel(v)) => {
                vxl_phys_terrain::voxel::contacts_sphere_voxel(v, center, radius, skin, out)
            }
            Some(ProviderEntry::Splat(f)) => f.contacts_sphere(id, center, radius, skin, out),
            Some(ProviderEntry::Mesh(m)) => m.contacts_sphere(id, center, radius, skin, out),
            None => false,
        }
    }
}
