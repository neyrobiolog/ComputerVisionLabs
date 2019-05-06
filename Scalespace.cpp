#include "scalespace.h"

#include <qmath.h>

ScaleSpace::ScaleSpace(const int numOctavs):
    _octavs(vector<vector<ScaleLevel>>(numOctavs)){}

ScaleSpace::ScaleSpace(const Matrix &initMatrix,const int octavNum):
    ScaleSpace(octavNum)
{
    calculate(initMatrix);
}

int ScaleSpace::octaveSize() const
{
    return _layerSize;
}

double ScaleSpace::sigma0() const
{
    return sigmaA;
}

const vector<vector<ScaleLevel> > & ScaleSpace::octavs() const
{
    return _octavs;
}

// Difference of Gaussian (DoG) - строим DOG то бишь
ScaleSpace ScaleSpace::computeDiffs() const
{
    // Новая пирамида - DOG
    ScaleSpace diffs(_octavs.size());

    for (int octav = 0; octav < _octavs.size(); octav++)
    {
        for (int layer = 0; layer < _layerSize + 2; layer++)
        {
            ScaleLevel diff;
            auto & m1 = _octavs[octav][layer].matrix;
            auto & m2 = _octavs[octav][layer+1].matrix;

            // D = g(x,y,ksigma) - g(x,y,sigma)
            diff.matrix = m1.compute(m2, std::minus<double>());

            diff.sigma = _octavs[octav][layer].sigma;
            diff.efectSigma = _octavs[octav][layer].efectSigma;
            diffs._octavs[octav].emplace_back(move(diff));
        }
    }

    return diffs;
}

// Поиск блобов
vector<Blob> ScaleSpace::searchBlobs() const
{
    vector<Blob> result;

    for (int octav = 0; octav < _octavs.size(); octav++)
    {
        for (int layer = 1; layer < _octavs[octav].size()-1; layer++)
        {
            auto & oLayer = _octavs[octav][layer];
            for (int i = 0; i < _octavs[octav][layer].matrix.width(); i++)
            {
                for (int j = 0; j < oLayer.matrix.height(); j++)
                {
                    if (checkExtremum(octav, layer, i, j))
                    {
                        result.emplace_back(Blob(i, j, octav, layer, oLayer.sigma));
                    }
                }
            }
        }
    }

    return result;
}

// Поиск локального максимума
bool ScaleSpace::checkExtremum(const int octav, const int level, const int i, const int j) const
{
    bool min = true;
    bool max = true;
    double center = _octavs[octav][level].matrix.get(i, j);

    if (std::abs(center) < EPS)
        return false;

    // Ищем min/max в 3D окрестности
    for (int x = -1; x < 2; x++)
    {
        for (int y = -1; y < 2; y++)
        {
            for (int z = -1; z < 2; z++)
            {
                if (!(x || y || z))
                    continue;

                double value = _octavs[octav][level+z].matrix.
                                get(i + x, j + y, Matrix::Border::COPIED);

                if (value > center) max = false;
                if (value < center) min = false;

                if (!(min || max))
                    return false;
            }
        }
    }

    return  min ^ max;
}

// Уточнение блобов
vector<Blob> BlobFilter::filter(const vector<Blob> & blobs, ScaleSpace & space) const
{
    vector<Blob> result;
    const auto & oct = space.octavs();

    for (const Blob & each:blobs)
    {
        double k = pow(2, each.octav);

        auto haris = CornerDetectors().computeHaris(oct[each.octav][each.layer].matrix, each.x, each.y, each.sigma*M_SQRT2);

        if (haris > treshold)
        {
            result.emplace_back(Blob(each));
        }
    }
    return result;
}

// Построение пирамиды Гаусса с нахлестом
void ScaleSpace::calculate(const Matrix & m)
{
    ScaleLevel initLevel;
    initLevel.sigma = sigmaA;
    initLevel.matrix = move(gauss(m, sqrt(pow(sigmaA,2) - pow(startSigma,2))));
    initLevel.efectSigma = sigmaA;
    _octavs[0].emplace_back(move(initLevel));

    for (int i = 0; i < _octavs.size(); i++)
    {
        for (int j = 0; j < _layerSize + 3; j++)
        {
            ScaleLevel level;

            if (j == 0)
            {
                if (i == 0)
                    continue;

                level.sigma = sigmaA;
                level.efectSigma = k * _octavs[i-1][_layerSize -1].efectSigma;
                level.matrix = _octavs[i-1][_layerSize - 1].matrix.compress();
            }

            else
            {
                auto &prevL = _octavs[i][j - 1];
                level.sigma = k * prevL.sigma;
                level.efectSigma = k * prevL.efectSigma;
                double delta = sqrt(pow(level.sigma, 2) - pow(prevL.sigma, 2));
                level.matrix = gauss(prevL.matrix, delta);
            }
            _octavs[i].emplace_back(move(level));
        }
    }
}

// Построение дескрипторов на основе блобов
vector<Descriptor> SIDiscrBuilder::build(const Matrix &m)
{
    // Строим пирамиду Гаусса (LoG)
    auto space = ScaleSpace(m, 5);
    // Строим DoG и блобы
    auto blobs = space.computeDiffs().searchBlobs();

    double treshold = 15;
    int curOct = -1;
    int curLayer = -1;

    vector<Point> points;
    vector<Descriptor> descriptors;
    Matrix derX;
    Matrix derY;
    int idx = 0;

    // Идём по всем блобам
    while (true && blobs.size() > 0)
    {
        const Blob & each = blobs[idx++];

        // Если список блобов не закончился, или не перешли в другую октаву/уровень
        if (curOct != each.octav || curLayer != each.layer || idx == blobs.size())
        {
            // Обработываем все блобы, что были собраны ранее
            if (points.size() > 0)
            {
                // Записали сигму, сформировали дексриптор
                auto sigma = space.octavs()[curOct][curLayer].sigma;
                auto descr = DescrBuilder(points, derX, derY, sigma, space.sigma0()).build();

                double scale = pow(2, curOct);

                // Работаем с дексриптором
                for (auto & eachDescr:descr)
                {
                    eachDescr.x *= scale;
                    eachDescr.y *= scale;
                    eachDescr.sigma = sigma * scale;
                    eachDescr.rad = sigma * scale * M_SQRT2;
                }

                descriptors.insert(descriptors.end(), descr.begin(), descr.end());
                points.clear();
            }

            // Если больше нет блобов, выходим
            if (idx == blobs.size())
                break;

            // Если ещё есть блобы, пересчитываем производные для Харриса в текущей матрице и работаем в новом ур/окт
            curOct = each.octav;
            curLayer = each.layer;
            auto & m = space.octavs()[curOct][curLayer].matrix;
            derX = m.convolution(KernelFactory::sobelX(),Matrix::Border::COPIED);
            derY = m.convolution(KernelFactory::sobelY(),Matrix::Border::COPIED);
        }

        double haris = CornerDetectors().computeHaris(each.x, each.y, each.sigma * M_SQRT2, derX, derY);

        if (haris > treshold)
        {
            points.emplace_back(Point(each.x, each.y, 0));
        }
    }

    return descriptors;
}

Matrix ScaleSpace::gauss(const Matrix &matrix,const double sigma) const
{
    return matrix
           .convolution(KernelFactory::createGaussX(sigma),Matrix::Border::COPIED)
           .convolution(KernelFactory::createGaussY(sigma),Matrix::Border::COPIED);
}

PointMatcher::PointMatcher(const double eps):eps(eps){}

// Матчинг: Поиск соответствия между изображениями
vector<pair<Point, Point> > PointMatcher::match(const Matrix &m1, const Matrix &m2, const bool withScale) const
{
    // Строим дескрипторы
    auto descrM1 = withScale?SIDiscrBuilder::build(m1):DescrBuilder(m1).build();
    auto descrM2 = withScale?SIDiscrBuilder::build(m2):DescrBuilder(m2).build();
    vector<pair<Point, Point>> samePoints;

    // Идём по дескрипторам первого изображения
    for (Descriptor & each:descrM1)
    {
        bool first = true;
        int descrIdx = -1;
        double minRo = 0;

        // Идём по дескрипторам второго изображения
        for (int idx = 0; idx < descrM2.size(); idx++)
        {
            double sum = 0;
            const Descriptor & each1 = descrM2[idx];

            // Подсчитываем расстояние по Эвклидовой формуле
            for (int i = 0; i < each.data.size(); i++)
            {
                sum += (each.data[i] - each1.data[i]) * (each.data[i] - each1.data[i]);
            }
            sum = sqrt(sum);

            if (sum < eps)
            {
                if (!first)
                {
                    if (minRo < sum)
                        descrIdx = idx;
                    break;
                }
                first = false;
                descrIdx = idx;
                minRo = sum;
            }
        }

        if (descrIdx > -1)
        {
            auto & d2 = descrM2[descrIdx];
            auto p1 = Point(each.x, each.y, 0);
            auto p2 = Point(d2.x, d2.y, 0);
            p1.angle = each.angle;
            p1.rad = each.rad;
            p1.sigma = each.sigma;
            p2.angle = d2.angle;
            p2.rad = d2.rad;
            p2.sigma = d2.sigma;
            samePoints.emplace_back(pair<Point,Point>(p1, p2));
            descrM2.erase(descrM2.begin() + descrIdx);
        }
    }
    return samePoints;
}
