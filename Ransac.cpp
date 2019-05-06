#include "ransac.h"

/*
#include <gsl/blas/gsl_blas.h>
#include <gsl/linalg/gsl_linalg.h>

Ransac::Ransac(){}

// Singular Value Decomposition
vector<double> Ransac::computeTrans(const vector<pair<Point, Point> > &pairs, const vector<int> &idx, const int size) const
{
    // Формируем страшную матрицу m, она же А (2*n x 9)
    gsl_matrix * m = gsl_matrix_calloc(2 *  size, M_WIDTH);
    for (int i = 0; i < size; i++)
    {
        auto & p = pairs[idx[i]];
        auto & d1 = p.first;
        auto & d2 = p.second;
        int row1 = 2*i;
        int row2 = 2*i+1;
        gsl_matrix_set(m, row1, 0, d1.x);
        gsl_matrix_set(m, row1, 1, d1.y);
        gsl_matrix_set(m, row1, 2, 1);
        gsl_matrix_set(m, row1, 3, 0);
        gsl_matrix_set(m, row1, 4, 0);
        gsl_matrix_set(m, row1, 5, 0);
        gsl_matrix_set(m, row1, 6, - d2.x * d1.x);
        gsl_matrix_set(m, row1, 7, - d2.x * d1.y);
        gsl_matrix_set(m, row1, 8, - d2.x);

        gsl_matrix_set(m, row2, 0, 0);
        gsl_matrix_set(m, row2, 1, 0);
        gsl_matrix_set(m, row2, 2, 0);
        gsl_matrix_set(m, row2, 3, d1.x);
        gsl_matrix_set(m, row2, 4, d1.y);
        gsl_matrix_set(m, row2, 5, 1);
        gsl_matrix_set(m, row2, 6, - d2.y * d1.x);
        gsl_matrix_set(m, row2, 7, - d2.y * d1.y);
        gsl_matrix_set(m, row2, 8, - d2.y);
    }

    // Транспонированная матрица m
    gsl_matrix* tr = gsl_matrix_calloc(M_WIDTH, 2 *  size);
    gsl_matrix_transpose_memcpy(tr, m);

    // Произведение матриц (m x mT)
    gsl_matrix* pr = gsl_matrix_calloc(M_WIDTH, M_WIDTH);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, tr, m, 0.0, pr);

    // Собственно SVD
    gsl_matrix* u = gsl_matrix_calloc(M_WIDTH, M_WIDTH);
    gsl_vector* sigma = gsl_vector_calloc(M_WIDTH);
    gsl_vector* buffer = gsl_vector_calloc(M_WIDTH);
    gsl_linalg_SV_decomp(pr, u, sigma, buffer);

    // Ищем в решении минимальное значение (колонку)
    auto idxMin = gsl_vector_min_index(sigma);
    gsl_vector_view singular = gsl_matrix_column(u, idxMin);

    // Формируем результат
    vector<double> h;
    for (int i = 0; i < M_WIDTH; i++)
    {
        h.push_back(gsl_vector_get(&singular.vector, i));
    }

    // Освободили память
    gsl_matrix_free(m);
    gsl_matrix_free(pr);
    gsl_matrix_free(u);
    gsl_vector_free(sigma);
    gsl_vector_free(buffer);

    return h;
}

// Нормализация
vector<double> Ransac::normalize(const vector<double> &v, const double r) const
{
    vector<double> result(v.size());
    std::transform(v.begin(), v.end(), result.begin(), Utils::div(r));
    return result;
}

double Ransac::computeError(const vector<double> &h,
    const pair<Point, Point> &pair, const int idx) const
{
    return h[idx] * pair.first.x + h[idx + 1] * pair.first.y + h[idx + 2];
}

// Вычисление трансформаций
vector<double> Ransac::searchTransform(
        const vector<pair<Point, Point> > &pairs) const
{
    vector<int> inliers;
    vector<int> idx(pairs.size());
    iota(idx.begin(), idx.end(), 0);

    for (int i = 0; i < LIMIT_ITER; i++)
    {
        random_shuffle(idx.begin(), idx.end());
        auto h = computeTrans(pairs, idx, COUNT);
        h = normalize(h, h[8]);

        vector<int> founded;	// Вектор "согласных" с гипотезой точек

        for (int j = 0; j < pairs.size(); j++)
        {
            // Вычисляем "согласие" точек - расстояние до линии
            auto div = computeError(h, pairs[j], 6);
            auto x = computeError(h, pairs[j], 0)/div;
            auto y = computeError(h, pairs[j], 3)/div;
            auto err = hypot(x - pairs[j].second.x, y - pairs[j].second.y);

            // Проверяем пороговое значение "согласия"
            if (err < EPS)
            {
                founded.push_back(j);
            }
        }

        // Записываем наибольшее значение inliers
        if (inliers.size() < founded.size())
        {
            inliers.clear();
            inliers.insert(inliers.end(),founded.begin(),founded.end());
        }

    }
    auto h = computeTrans(pairs, inliers, inliers.size());
    return normalize(h, h[8]);
}

*/
