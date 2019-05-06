#include "descriptor.h"

#define M_PI 3.14

Descriptor::Descriptor(const int x, const int y, const double angle):
    x(x),y(y),angle(angle){}

Descriptor::Descriptor(const int x, const int y,
                       const vector<double> &data,const double angle):
    Descriptor(x,y,angle)
{
    for(double each:data){
        this->data.push_back(each);
    }
}

DescrBuilder::DescrBuilder(const Matrix &m)
{
    sobelX = m.convolution(KernelFactory::sobelX(),Matrix::Border::COPIED);
    sobelY = m.convolution(KernelFactory::sobelY(),Matrix::Border::COPIED);
    points = CornerDetectors().detect(m,Algorithm::HARIS);
    points = PointFileter(m.width()*m.height(),maxNumPoints).filter(points);
    sigma0 = 2.0;
    sigma = 2.0;
}

DescrBuilder::DescrBuilder(
        const vector<Point> & points, Matrix & derX,
            Matrix & derY, const double sigma, const double sigma0):
                                    points(points),sigma(sigma),sigma0(sigma0)
{
    sobelX = move(derX);
    sobelY = move(derY);
}

// Построение дескрипторов
vector<Descriptor> DescrBuilder::build() const
{
    auto descriptors = vector<Descriptor>();

    for (auto &each : points)
    {
        auto data = computeData(each, 0, numRotateHist, sizeRotateHist, numRotateBins);
        auto maxh = *max_element(data.begin(), data.end());
        double angles[3];
        int nangles = 0;

        for (int i = 0; i < data.size(); i++)
        {
            auto h0 = data[i];
            auto hm = data[(i - 1 + data.size())%data.size()];
            auto hp = data[(i + 1 + data.size())%data.size()];

            if (h0 > 0.8 * maxh && h0 > hm && h0 > hp)
            {
                auto di = -0.5 * (hp - hm) / (hp + hm - 2 * h0);
                auto th = 2  * M_PI * (i + di + 0.5) / data.size();
                angles[nangles++] = th;
                if (nangles == 3)
                    break;
            }
        }

        // Больше двух ориентаций не имеет смысла
        if (nangles > 2)
            continue;

        for (int i = 0; i < nangles; i++)
        {
            descriptors.emplace_back(normilizeAll(
                                       Descriptor(each.x,each.y,
                                            computeData(each, angles[i],
                                                numDescrHist, sizeDescrHist, numDescrBins), angles[i])));
        }
    }
    return descriptors;
}

// Формирование дескриптора
vector<double> DescrBuilder::computeData(const Point &p, const double rotateAngle, const int numHist,
                                         const int sizeHist, const int numBins) const
{
    /*
    int size = sizeHist * (sigma/sigma0);
    int sizeArea = numHist * size; // Размер окрестности
    auto data = vector<double>(numHist * numHist * numBins, 0.0);
    auto aSin = sin(rotateAngle);
    auto aCos = cos(rotateAngle);

    for (int i = 0; i < sizeArea; i++)
    {
        for (int j = 0; j < sizeArea; j++)
        {
            int cx = i;
            int cy = j;

            // Для каждой точки (x, y) вычисляем (x', y')
            int x = (cx * aCos + cy * aSin);
            int y = (-cx * aSin + cy * aCos);

            // Проверка на выход за границы окрестности
            if (x >= sizeArea|| y >= sizeArea || x < 0 || y < 0)
            {
                continue;
            }

            // Строим широкую гистограмму распределения ориентации градиентов в области
            double dx = sobelX.get(p.x + cx, p.y + cy, Matrix::Border::COPIED);
            double dy = sobelY.get(p.x + cx, p.y + cy, Matrix::Border::COPIED);
            // Величина градиента
            double magnitud = sqrt(dx * dx + dy * dy) * Utils::gauss(cx, cy, sigma);
            // Ориентация градиента
            double phi = atan2(dx, dy) + M_PI - rotateAngle;

            phi = fmod(phi + 4 * M_PI - M_PI/numBins, 2 * M_PI);

            double num = phi/(2 * M_PI/numBins);

            int binIdx1 = ((int)floor(num))%numBins;
            int binIdx2 = ((int)(floor(num) + 1))%numBins;

            //нашли индексы текущей гистограммы
            int xBin = x/size;
            int yBin = y/size;

            if (numHist > 1)
            {
                // Нашли центр текущей гистограммы
                int xCenter = (xBin) * sizeHist + sizeHist/2;
                int yCenter = (yBin) * sizeHist + sizeHist/2;

                // Перебираем соседние гистограммы на возможность добавить в них взвешанные значение
                for (int k = -1; k < 2; k++)
                {
                    for (int t = -1; t < 2; t++)
                    {
                        // Соседний центр относительно текущей гистограммы
                        int sameX = xCenter + k * sizeHist;
                        int sameY = yCenter + t * sizeHist;
                        int sameCX = xBin + k;
                        int sameCY = yBin + t;

                        // Получение невалидного индекса бина
                        if (sameCX < 0 || sameCY < 0 || sameCX >= numHist || sameCY >= numHist)
                            continue;

                        // Проверка на выход за границы окрестности
                        if (sameX >= sizeArea || sameY >= sizeArea || sameX < 0 || sameY < 0)
                            continue;

                        // Cлишком далеко от центра рассматриваемого бина
                        if (sameX + sizeHist < x || x < sameX - sizeHist)
                            continue;
                        if (sameY + sizeHist < y || y < sameY - sizeHist)
                            continue;

                        double w0 = 1.0 - (abs(sameX - x)/(double)sizeHist);
                        double w1 = 1.0 - (abs(sameY - y)/(double)sizeHist);

                        int hIdx = ((sameCX) * numHist + (sameCY)) * numBins;
                        data[hIdx + binIdx1] += w0 * w1*magnitud * (ceil(num) - num);
                        data[hIdx + binIdx2] += w0 * w1*magnitud * (num - floor(num));
                    }
                }
            }

            else
            {
                int hIdx = (xBin * numHist + yBin) * numBins;
                data[hIdx + binIdx1] += magnitud * (ceil(num) - num);
                data[hIdx + binIdx2] += magnitud * (num - floor(num));
            }
        }
    }
    return data;*/

    int size = sizeHist * (sigma/sigma0);
    int sizeArea = numHist * size;
    auto data = vector<double>(numHist * numHist * numBins, 0.0);
    auto aSin = sin(rotateAngle);
    auto aCos = cos(rotateAngle);

    for(int i = 0; i < sizeArea; i++)
    {
        for(int j = 0; j < sizeArea; j++)
        {

            int cx = i;
            int cy = j;

            // Для каждой точки (x, y) вычисляем (x', y')
            int x = (cx * aCos + cy * aSin);
            int y = (-cx * aSin + cy * aCos);

            // Проверка на выход за границы окрестности
            if (x >= sizeArea || y >= sizeArea || x < 0 || y < 0)
                continue;

            // Строим широкую гистограмму распределения ориентации градиентов в области
            double dx = sobelX.get(p.x + cx, p.y + cy, Matrix::Border::COPIED);
            double dy = sobelY.get(p.x + cx, p.y + cy, Matrix::Border::COPIED);
            // Величина градиента
            double magnitud = sqrt(dx * dx + dy * dy) * Utils::gauss(cx, cy, sigma);
            // Ориентация градиента
            double phi = atan2(dx, dy) + M_PI - rotateAngle;

            phi = fmod(phi + 4 * M_PI - M_PI/numBins, 2 * M_PI);

            double num = phi/(2 * M_PI/numBins);

            int binIdx1 = ((int)floor(num)) % numBins;
            int binIdx2 = ((int)(floor(num) + 1)) % numBins;

            // Hашли индексы текущей гистограммы
            int xBin = x/size;
            int yBin = y/size;

            if (numHist > 1)
            {
                // Нашли центр текущей гистограммы
                int xCenter = (x/size) * sizeHist + sizeHist/2;
                int yCenter = (y/size) * sizeHist + sizeHist/2;

                //  Перебираем соседние гистограммы на возможность добавить в них взвешанные значение
                for (int k = -1; k < 2; k++)
                {
                    for (int t = -1; t < 2; t++)
                    {
                        // Нашли соседний центр относительно текущей гистограммы
                        int sameX = xCenter + k * sizeHist;
                        int sameY = yCenter + t * sizeHist;
                        int sameCX = xBin + k;
                        int sameCY = yBin + t;

                        // Получили невалидный индекс бина
                        if (sameCX < 0 || sameCY < 0 || sameCX >= numHist || sameCY >= numHist)
                            continue;

                        // Проверили, не вышли ли за границы области
                        if (sameX >= sizeArea || sameY >= sizeArea || sameX < 0 || sameY < 0)
                            continue;

                        double w0;
                        // Соседняя гистограмма слева
                        if (sameX < x && x <= xCenter)
                            w0 = (x - sameX)/(double)sizeHist;
                        // Соседняя гистограмма справа
                        else if (xCenter < x && x < sameX)
                            w0 = (sameX - x)/(double)sizeHist;
                        // Центральная гистограмма
                        else if (sameX == xCenter)
                            w0 = abs(x - sameX)/(double)sizeHist;
                        else
                            continue;

                        double w1;
                        // Соседняя гистограмма ниже
                        if (sameY < y && y <= yCenter)
                            w1 = (y - sameY)/(double)sizeHist;
                        // Соседняя гистограмма выше
                        else if (xCenter < x && x < sameY)
                            w1 = (sameY - y)/(double)sizeHist;
                        // Центральная гистограмма
                        else if (sameX == xCenter)
                            w1 = abs(y - sameY)/(double)sizeHist;
                        else
                            continue;

                        int hIdx = ((sameCX) * numHist + (sameCY)) * numBins;
                        data[hIdx + binIdx1] += w0 * w1 * magnitud * (ceil(num) - num);
                        data[hIdx + binIdx2] += w0 * w1 * magnitud * (num - floor(num));
                    }
                }
            }

            else
            {
                int hIdx = (xBin * numHist + yBin) * numBins;
                data[hIdx + binIdx1] += magnitud * (ceil(num) - num);
                data[hIdx + binIdx2] += magnitud * (num - floor(num));
            }
        }
    }

    return data;
}

// Нормализуем - обрезаем по пороговому значению - опять нормализуем
Descriptor DescrBuilder::normilizeAll(const Descriptor &descr) const
{
    return normilize(filterTrash(normilize(descr)));
}

// Обрезание по пороговому значению
Descriptor DescrBuilder::filterTrash(const Descriptor &descr) const
{
    Descriptor result(descr.x,descr.y);
    result.rad = descr.rad;
    result.angle = descr.angle;

    for (double bin:descr.data)
    {
        result.data.push_back(min(treshold, bin));
    }

    return result;
}

// Нормализация
Descriptor DescrBuilder::normilize(const Descriptor &descr) const
{
    Descriptor result(descr.x,descr.y);
    result.rad = descr.rad;
    result.angle = descr.angle;
    double sum = 0;

    for (double each:descr.data)
    {
        sum += each * each;
    }
    sum = sqrt(sum);

    for (double each:descr.data)
    {
        result.data.push_back(each/sum);
    }
    return result;
}

// Ищем совпадающие дескрипторы для двух изображений (Lr4)
vector<pair<Point, Point> > PointSearcher::findSamePoints(const Matrix &m1, const Matrix &m2) const
{
    // Дескрипторы изображений
    auto descrM1 = DescrBuilder(m1).build();
    auto descrM2 = DescrBuilder(m2).build();

    vector<pair<Point, Point>> samePoints;

    // Перебираем все дескрипторы
    for(Descriptor & each:descrM1)
    {
        for(Descriptor & each1:descrM2)
        {
             double sum = 0;
             for (int i = 0; i < each.data.size(); i++)
             {
                 sum += (each.data[i] - each1.data[i]) * (each.data[i] - each1.data[i]);
             }
             sum = sqrt(sum);

             // Кладём дескриптор в вектор, если он удовлетворяет
             if (sum < eps)
             {
                 samePoints.emplace_back(
                      pair<Point, Point>(Point(each.x,each.y,0),
                                         Point(each1.x,each1.y,0)));
                 break;
             }
        }
    }
    return samePoints;
}

