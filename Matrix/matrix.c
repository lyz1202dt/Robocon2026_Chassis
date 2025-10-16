#include "matrix.h"

#include "matrix.h"
#include <math.h>  // 确保有sqrt和fabs声明

// 交换两个整数
static void swap_int(int* a, int* b) {
    int tmp = *a; *a = *b; *b = tmp;
}

// 交换矩阵A的两列
static void swap_cols(float A[M][N], int col1, int col2) {
    for (int i = 0; i < M; i++) {
        float tmp = A[i][col1];
        A[i][col1] = A[i][col2];
        A[i][col2] = tmp;
    }
}

// 计算列向量的2范数
static float col_norm(const float A[M][N], int col, int start_row) {
    float sum = 0.0;
    for (int i = start_row; i < M; i++) {
        sum += A[i][col] * A[i][col];
    }
    return sqrt(sum);
}

// 计算Householder向量和beta
static float compute_householder_vector(float v[], int len) {
    float sigma = 0.0;
    for (int i = 1; i < len; i++) sigma += v[i] * v[i];

    if (sigma == 0.0) {
        return 0.0;
    }

    float mu = sqrt(v[0]*v[0] + sigma);
    if (v[0] <= 0)
        v[0] = v[0] - mu;
    else
        v[0] = -sigma / (v[0] + mu);

    float beta = 2.0 * v[0] * v[0] / (sigma + v[0]*v[0]);

    for (int i = 1; i < len; i++) {
        v[i] /= v[0];
    }
    v[0] = 1.0;

    return beta;
}

int solve_linear_system_8x3(const float A_in[M][N], const float b_in[M][1], float x_out[N][1]) {
    // 复制A和b，因为会修改
    float A[M][N];
    float b[M];
    for (int i = 0; i < M; i++) {
        b[i] = b_in[i][0];  // 修改这里，访问二维数组元素
        for (int j = 0; j < N; j++) {
            A[i][j] = A_in[i][j];
        }
    }

    int col_perm[N] = {0, 1, 2};

    float col_norms[N];
    for (int j = 0; j < N; j++) {
        col_norms[j] = col_norm(A, j, 0);
    }

    float v[M];

    for (int k = 0; k < N; k++) {
        int max_col = k;
        float max_norm = col_norms[k];
        for (int j = k+1; j < N; j++) {
            if (col_norms[j] > max_norm) {
                max_norm = col_norms[j];
                max_col = j;
            }
        }
        if (max_col != k) {
            swap_cols(A, k, max_col);
            swap_int(&col_perm[k], &col_perm[max_col]);
            {
                float tmp = col_norms[k];
                col_norms[k] = col_norms[max_col];
                col_norms[max_col] = tmp;
            }
        }

        int len = M - k;
        for (int i = 0; i < len; i++) {
            v[i] = A[k + i][k];
        }

        float beta = compute_householder_vector(v, len);

        if (beta != 0.0) {
            for (int j = k; j < N; j++) {
                float w = 0.0;
                for (int i = 0; i < len; i++) {
                    w += v[i] * A[k + i][j];
                }
                w *= beta;

                for (int i = 0; i < len; i++) {
                    A[k + i][j] -= w * v[i];
                }
            }

            float w_b = 0.0;
            for (int i = 0; i < len; i++) {
                w_b += v[i] * b[k + i];
            }
            w_b *= beta;
            for (int i = 0; i < len; i++) {
                b[k + i] -= w_b * v[i];
            }
        }

        for (int j = k + 1; j < N; j++) {
            if (col_norms[j] != 0) {
                float val = A[k][j] / col_norms[j];
                float temp = 1.0 - val * val;
                if (temp < 0) temp = 0;
                col_norms[j] *= sqrtf(temp);
            }
        }
    }

    float z[N] = {0};
    for (int i = N - 1; i >= 0; i--) {
        float sum = 0.0;
        for (int j = i + 1; j < N; j++) {
            sum += A[i][j] * z[j];
        }
        if (fabsf(A[i][i]) < 1e-12) {
            return -1;
        }
        z[i] = (b[i] - sum) / A[i][i];
    }

    // 恢复列顺序，这里修改为访问二维数组元素
    for (int i = 0; i < N; i++) {
        x_out[col_perm[i]][0] = z[i];
    }

    return 0;
}
