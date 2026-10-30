#ifndef PLANAR_WHEEL_MATH_HPP
#define PLANAR_WHEEL_MATH_HPP

#include <cmath>

namespace Alg::CalculationBase::WheelMath
{
    // 轮驱动方向 angle、轮心坐标 x/y，均在车体坐标系中；scale 为车体几何尺度 m。
    // row * [vx, vy, scale*wz] 是轮缘沿驱动方向的速度 m/s。
    inline void Row(float angle, float x, float y, float scale, double row[3])
    {
        row[0] = std::cos(angle);
        row[1] = std::sin(angle);
        row[2] = (x * row[1] - y * row[0]) / scale;
    }

    inline void AddOuter(const double row[3], double normal[3][3])
    {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                normal[i][j] += row[i] * row[j];
    }

    inline void Accumulate(const double row[3], double value, double normal[3][3], double rhs[3])
    {
        AddOuter(row, normal);
        for (int i = 0; i < 3; ++i) rhs[i] += row[i] * value;
    }

    // 对称半正定 3x3 矩阵的伪逆乘 rhs；最多 24 次 Jacobi 旋转，无堆内存。
    // 舵角平行时矩阵会降秩，丢弃小于最大特征值 1e-6 的方向，只求可实现的部分。
    inline void Solve(const double normal[3][3], const double rhs[3], double result[3])
    {
        double a[3][3], vectors[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
        double scale = 0;
        for (int i = 0; i < 3; ++i)
        {
            result[i] = 0;
            if (normal[i][i] > scale) scale = normal[i][i];
            for (int j = 0; j < 3; ++j) a[i][j] = normal[i][j];
        }
        for (int iteration = 0; iteration < 24; ++iteration)
        {
            int p = 0, q = 1;
            for (int i = 0; i < 3; ++i)
                for (int j = i + 1; j < 3; ++j)
                    if (std::fabs(a[i][j]) > std::fabs(a[p][q])) { p = i; q = j; }
            if (std::fabs(a[p][q]) <= scale * 1e-12) break;
            const double tau = (a[q][q] - a[p][p]) / (2 * a[p][q]);
            const double t = std::copysign(1.0, tau) / (std::fabs(tau) + std::sqrt(1 + tau*tau));
            const double c = 1 / std::sqrt(1 + t*t), s = t*c;
            a[p][p] -= t * a[p][q];
            a[q][q] += t * a[p][q];
            a[p][q] = a[q][p] = 0;
            for (int k = 0; k < 3; ++k)
            {
                if (k != p && k != q)
                {
                    const double kp = a[k][p], kq = a[k][q];
                    a[k][p] = a[p][k] = c*kp - s*kq;
                    a[k][q] = a[q][k] = s*kp + c*kq;
                }
                const double vp = vectors[k][p], vq = vectors[k][q];
                vectors[k][p] = c*vp - s*vq;
                vectors[k][q] = s*vp + c*vq;
            }
        }
        double largest = 0;
        for (int i = 0; i < 3; ++i) if (a[i][i] > largest) largest = a[i][i];
        for (int k = 0; k < 3; ++k)
        {
            if (a[k][k] <= largest * 1e-6) continue;
            double projection = 0;
            for (int i = 0; i < 3; ++i) projection += vectors[i][k] * rhs[i];
            for (int i = 0; i < 3; ++i) result[i] += vectors[i][k] * projection / a[k][k];
        }
    }
}

#endif
