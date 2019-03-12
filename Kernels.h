#ifndef KERNELS_H
#define KERNELS_H
#include <array>
#include <vector>

// Частные поизводные по Собелю
static const std::vector<int> VsobelX { -1, 0, 1, -2, 0, 2, -1, 0, 1 };
static const std::vector<int> VsobelY { 1, 2, 1, 0, 0, 0, -1, -2, -1 };

// Частные поизводные по Прюитту
static const std::vector<int> VprewittX { -1, 0, 1, -1, 0, 1, -1, 0, 1 };
static const std::vector<int> VprewittY { -1, -1, -1, 0, 0, 0, 1, 1, 1 };

// Частные поизводные по Щарру
static const std::vector<int> VscharrX { 3, 10, 3, 0, 0, 0, -3, -10, -3 };
static const std::vector<int> VscharrY { 3, 0, -3, 10, 0, -10 , 3, 0, -3 };

#endif // KERNELS_H
