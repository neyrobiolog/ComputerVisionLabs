#ifndef CMATRIXV_H
#define CMATRIXV_H
#include <vector>
#include <algorithm>

template<typename T>

class Matrix
{
private:

    int m_columns;
    int m_rows;
    std::vector<T> m_matrix;

public:

    // Конструктор без параметров
    Matrix()
    {
        m_columns = 0;
        m_rows = 0;
        m_matrix.resize(0, 0);
    }

    // Конструктор с параметрами
    Matrix(int columns, int rows)
    {
        m_columns = columns;
        m_rows = rows;
        m_matrix.resize(columns * rows, 0);
    }

    // Декструктор
    ~Matrix()
    {
        m_columns = 0;
        m_rows = 0;
        m_matrix.clear();
    }

    Matrix& operator= (const Matrix& matrix)
    {
        m_columns = matrix.m_columns;
        m_rows = matrix.m_rows;
        m_matrix.resize(m_rows * m_columns);
        std::copy(matrix.m_matrix.begin(), matrix.m_matrix.end(), m_matrix.begin());
        return *this;
    }

    Matrix& operator= (Matrix&& matrix)
    {
        if (this != &matrix)
        {
            m_matrix.clear();
            m_columns = 0;
            m_rows = 0;

            m_columns = std::move(matrix.m_columns);
            m_rows = std::move(matrix.m_rows);
            m_matrix = std::move(matrix.m_matrix);

        }
        return *this;
    }

    // Конструктор перемещения
    Matrix(int columns, int rows, const std::vector<T>& vectorForCopy )
    {
        m_columns = columns;
        m_rows = rows;
        m_matrix = std::move(vectorForCopy);
    }

    // Столбцы
    int getColumns() const  { return m_columns; }
    void setColumns( int columns ) { m_columns = columns; }

    // Строки
    int getRows() const { return m_rows; }
    void setRows(int rows)   { m_rows = rows; }

    std::vector<T> getMatrix()  { return m_matrix; }

    T getItem(int currentColumns, int currentRows) const
    {
        return m_matrix[currentColumns * m_rows + currentRows];
    }

    void setItem(int currentColumns, int currentRows, T value)
    {
        m_matrix[currentColumns * m_rows + currentRows] = value;
    }

    // Нормализация матрицы
    void normalize()
    {
        auto it_min = std::min_element( m_matrix.begin(), m_matrix.end() );
        T min = *it_min;
        if (min < 0)
        {
            for (size_t i = 0; i < m_matrix.size(); i++)
            {
                m_matrix[i] += min;
            }
        }

        auto it_max = std::max_element( m_matrix.begin(), m_matrix.end() );
        T max = *it_max;
        if (max > 255)
            for (size_t i = 0; i < m_matrix.size(); i++)
            {
                m_matrix[i] /= max;
                m_matrix[i] *= 255;
            }
    }

    // Изменение размера
    void resize(int columns, int rows, const std::vector<T>& vectorForCopy)
    {
        m_columns = columns;
        m_rows = rows;
        m_matrix = std::move(vectorForCopy);
    }

};

#endif // CMATRIXV_H
