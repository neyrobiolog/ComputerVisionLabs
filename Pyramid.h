#ifndef PYRAMID_H
#define PYRAMID_H
#include "Image.h"
#include <vector>

class Pyramid
{
private:

    int m_octaves;                      // Число октав пирамиды
    int m_sclaes;                       // Число масштабов пирамиды
    float m_evectifSigma;               // Итоговое значение сигмы
    std::vector<float> m_sigmas;        // Вектор сигм всех октав
    std::vector<Image> m_pyramidImage; // Вектор изображений всех октав

public:

    Pyramid();                                             // Конструктор по умолчанию
    Pyramid(int octaves, float sigmaZero, int scales);  // Конструктор с параметрами

    // Задать изображение и его сигму в текущей октаве
    void setImageInOctaves(Image& myImage, int octava, float sigma)
    {
        m_pyramidImage[octava] = myImage;
        m_sigmas[octava] = sigma;
        m_evectifSigma *= sigma;
    }

    // Получить изображение из октавы
    Image getImageInOctaves(int octava)
    {
        return m_pyramidImage[octava];
    }

    // Получить значение сигмы из октавы
    float getSigmasInOctaves(int octava)
    {
        return m_sigmas[octava];
    }

    // Получить количество октав в пирамиде
    int getOctaves()
    {
        return m_octaves;
    }
};

#endif // PYRAMID_H
