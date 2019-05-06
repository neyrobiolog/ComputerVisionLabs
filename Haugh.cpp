#include "haugh.h"

#include <cstddef>
#include <memory>
#include <type_traits>
#include <utility>

namespace std {
    template<class T> struct _Unique_if {
        typedef unique_ptr<T> _Single_object;
    };

    template<class T> struct _Unique_if<T[]> {
        typedef unique_ptr<T[]> _Unknown_bound;
    };

    template<class T, size_t N> struct _Unique_if<T[N]> {
        typedef void _Known_bound;
    };

    template<class T, class... Args>
        typename _Unique_if<T>::_Single_object
        make_unique(Args&&... args) {
            return unique_ptr<T>(new T(std::forward<Args>(args)...));
        }

    template<class T>
        typename _Unique_if<T>::_Unknown_bound
        make_unique(size_t n) {
            typedef typename remove_extent<T>::type U;
            return unique_ptr<T>(new U[n]());
        }

    template<class T, class... Args>
        typename _Unique_if<T>::_Known_bound
        make_unique(Args&&...) = delete;
}

#define M_PI 3.14

Hough::Hough(const int width1, const int height1, const int width2, const int heigth2,
             const vector<pair<Point, Point>> pairs):pairs(pairs){
    centerX = width1 / 2;
    centerY = height1 / 2;
    numBinsX = width1 / coordBins;
    numBinsY = heigth2 / coordBins;
}

// Преобразование Хафа
Hough::HoughValue Hough::computeHaugh()
{
    unique_ptr<double []> scales = make_unique<double []>(scaleBins + 1);
    std::generate(scales.get(),scales.get()+scaleBins,Progression(initScale,scaleStap));

    unique_ptr<double[]> mid = make_unique<double[]>(scaleBins);
    for (int i = 0; i < scaleBins; i++)
    {
        mid[i] = (scales[i] + scales[i + 1]) / 2;
    }

    // Аккумулятор
    const int size = numBinsX * numBinsY * angleBins * scaleBins;
    unique_ptr<int[]> acc = make_unique<int[]>(size);
    std::fill(acc.get(), acc.get() + size, 0);

    // Переменные для голосования
    int max = 0;
    int i = 0, j = 0, angl = 0, scale = 0;
    vector<int> idxI, idxJ, idxScale, idxAngl;  // Измерения параметров

    // Построение 4х-мерного пространства параметров объекта
    for (auto & pair : pairs)
    {
        int idx1 = pair.first.x - centerX,
            idy1 = pair.first.y - centerY;
        double length = hypot(idx1, idy1);
        double angle = pair.second.angle - pair.first.angle + atan2(idy1, idx1);

        double sc = pair.second.sigma / pair.first.sigma;

        double dx = length * sc * std::cos(angle),
               dy = length * sc * std::sin(angle);

        double idx2 = round(pair.second.x - dx)/(double)coordBins,
               idy2 = round(pair.second.y - dy)/(double)coordBins;

        double angle2 = fmod(pair.second.angle - pair.first.angle + 2 * M_PI, 2 * M_PI);

        // Первое измерение на координаты (X)
        int idx = std::floor(idx2);
        idxI.push_back(idx2);
        if (idx2 > 0.5 && idx2 < coordBins - 0.5)
        {
            if (idx2 - idx >= 0.5)
                idxI.push_back(idx + 1);

            else
                idxI.push_back(idx - 1);
        }

        // Второе измерение на координаты (Y)
        int idy = std::floor(idy2);
        idxJ.push_back(idy2);
        if (idy2 > 0.5 && idy2 < coordBins - 0.5)
        {
            if (idy2 - idy >= 0.5)
                idxJ.push_back(idy + 1);

            else
                idxJ.push_back(idy - 1);
        }

        // Измерение на ориентацию объекта
        double a = angle2 / angleBins;
        int aIdx = std::floor(a);
        idxAngl.push_back(aIdx);
        if (a - aIdx >= 0.5)
            idxAngl.push_back((aIdx + 1) % angleBins);

        else
            idxAngl.push_back((aIdx - 1 + angleBins) % angleBins);

        // Измерение на масштаб объекта
        for (int idx = 0; idx < scaleBins; idx++)
        {
            if (scales[idx] <= sc && sc < scales[idx + 1])
            {
                idxScale.push_back(idx);

                if (sc > mid[idx])
                {
                    if (idx <= scaleBins)
                        idxScale.push_back(idx+1);
                }

                else if (idx > 0)
                    idxScale.push_back(idx - 1);

                break;
            }
        }

        // Подсчитываем голоса
        for (int I : idxI)
        {
            for (int J : idxJ)
            {
                for (int A : idxAngl)
                {
                    for (int S : idxScale)
                    {
                        int accIdx = I * numBinsY * angleBins * scaleBins +
                                     J * angleBins * scaleBins +
                                     A * scaleBins +
                                     S;
                        acc[accIdx]++;

                        // Если в текущем анализе голосов больше всего - запишем их
                        if (acc[accIdx] > max)
                            max = acc[accIdx], i = I,j = J,angl = A, scale = S;
                    }
                }
            }
        }

        idxI.clear();
        idxJ.clear();
        idxAngl.clear();
        idxScale.clear();
    }

    return HoughValue((i + 0.5) * coordBins,
                      (j + 0.5) * coordBins,
                      (angl + 0.5) * M_PI / angleBins,
                      mid[scale]);
}
