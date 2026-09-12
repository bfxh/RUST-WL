//! 临时诊断（验收后删除）：小规模顺序插入的树形观察。

use vxl_phys_broad::bvh::DynamicBvh;
use vxl_phys_broad::Aabb;
use vxl_phys_core::Vec3;

fn aabb_at(x: f32, y: f32, half: f32) -> Aabb {
    let c = Vec3::new(x, y, 0.0);
    Aabb {
        min: c - Vec3::splat(half),
        max: c + Vec3::splat(half),
    }
}

#[test]
fn diag_small_tree_shape() {
    // 行优先网格 200 体（模拟 100×100 静态瓦片布局的小样）。
    let mut t = DynamicBvh::new(0.02);
    for k in 0..200u32 {
        let x = (k % 20) as f32 * 1.0 - 10.0;
        let z = (k / 20) as f32 * 1.0 - 5.0;
        t.insert(k, aabb_at(x, 0.5, 0.5));
        if (k + 1) % 40 == 0 {
            t.validate();
            println!("网格插入 n={} 树高={}", k + 1, t.root_height());
        }
    }
    // 同样 200 体的伪随机散布。
    let mut t2 = DynamicBvh::new(0.02);
    for k in 0..200u32 {
        let x = ((k.wrapping_mul(2654435761)) % 1000) as f32 / 1000.0 * 40.0 - 20.0;
        let z = ((k.wrapping_mul(40503)) % 1000) as f32 / 1000.0 * 40.0 - 20.0;
        t2.insert(k, aabb_at(x, 0.5, 0.5));
        if (k + 1) % 40 == 0 {
            t2.validate();
            println!("随机插入 n={} 树高={}", k + 1, t2.root_height());
        }
    }
}
