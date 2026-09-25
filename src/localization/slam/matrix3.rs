//! The few 3x3 matrix operations SLAM needs on `(x, y, heading)`
//! covariances and information matrices - Karto's `Matrix3`.

/// A 3x3 matrix, row-major.
pub type Matrix3 = [[f64; 3]; 3];

pub fn identity() -> Matrix3 {
    diagonal(1.0, 1.0, 1.0)
}

pub fn diagonal(a: f64, b: f64, c: f64) -> Matrix3 {
    [[a, 0.0, 0.0], [0.0, b, 0.0], [0.0, 0.0, c]]
}

pub fn mul(a: &Matrix3, b: &Matrix3) -> Matrix3 {
    let mut out = [[0.0; 3]; 3];
    for (i, row) in out.iter_mut().enumerate() {
        for (j, cell) in row.iter_mut().enumerate() {
            *cell = (0..3).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    out
}

pub fn mul_vec(a: &Matrix3, v: &[f64; 3]) -> [f64; 3] {
    [0, 1, 2].map(|i| (0..3).map(|k| a[i][k] * v[k]).sum())
}

pub fn transpose(a: &Matrix3) -> Matrix3 {
    [0, 1, 2].map(|i| [0, 1, 2].map(|j| a[j][i]))
}

pub fn add(a: &Matrix3, b: &Matrix3) -> Matrix3 {
    [0, 1, 2].map(|i| [0, 1, 2].map(|j| a[i][j] + b[i][j]))
}

/// The inverse, or `None` if `a` is (numerically) singular.
pub fn inverse(a: &Matrix3) -> Option<Matrix3> {
    let cofactor =
        |r0: usize, r1: usize, c0: usize, c1: usize| a[r0][c0] * a[r1][c1] - a[r0][c1] * a[r1][c0];
    let c00 = cofactor(1, 2, 1, 2);
    let c01 = -cofactor(1, 2, 0, 2);
    let c02 = cofactor(1, 2, 0, 1);
    let det = a[0][0] * c00 + a[0][1] * c01 + a[0][2] * c02;
    if !det.is_finite() || det.abs() < 1e-300 {
        return None;
    }
    let adjugate = [
        [c00, -cofactor(0, 2, 1, 2), cofactor(0, 1, 1, 2)],
        [c01, cofactor(0, 2, 0, 2), -cofactor(0, 1, 0, 2)],
        [c02, -cofactor(0, 2, 0, 1), cofactor(0, 1, 0, 1)],
    ];
    Some(adjugate.map(|row| row.map(|value| value / det)))
}

/// A rotation by `angle_rad` about the heading axis: rotates `(x, y)`,
/// leaves the heading alone.
pub fn rotation(angle_rad: f64) -> Matrix3 {
    let (sin, cos) = angle_rad.sin_cos();
    [[cos, -sin, 0.0], [sin, cos, 0.0], [0.0, 0.0, 1.0]]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_close(a: &Matrix3, b: &Matrix3) {
        for i in 0..3 {
            for j in 0..3 {
                assert!((a[i][j] - b[i][j]).abs() < 1e-9, "{a:?} != {b:?}");
            }
        }
    }

    #[test]
    fn a_matrix_times_its_inverse_is_the_identity() {
        let a = [[4.0, 1.0, 0.5], [1.0, 3.0, 0.2], [0.5, 0.2, 2.0]];
        let inverse = inverse(&a).expect("a is invertible");
        assert_close(&mul(&a, &inverse), &identity());
    }

    #[test]
    fn a_singular_matrix_has_no_inverse() {
        let a = [[1.0, 2.0, 3.0], [2.0, 4.0, 6.0], [0.0, 0.0, 1.0]];
        assert_eq!(inverse(&a), None);
    }

    #[test]
    fn rotating_by_an_angle_and_back_is_the_identity() {
        let r = rotation(0.7);
        assert_close(&mul(&r, &transpose(&r)), &identity());
        assert_close(&rotation(-0.7), &transpose(&r));
    }
}
