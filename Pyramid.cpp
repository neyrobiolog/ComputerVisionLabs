#include "Pyramid.h"

// Конструктор по умолчанию
Pyramid::Pyramid()
{
    m_octaves = 0;
    m_sclaes = 0;
}

// Конструктор с параметрами
Pyramid::Pyramid(int octaves, float sigmaZero, int scales)
{
    m_octaves = octaves;
    m_pyramidImage.resize(m_octaves, Image());
    m_sigmas.resize(m_octaves, 0);
    m_evectifSigma = sigmaZero;
    m_sclaes = scales;
}
