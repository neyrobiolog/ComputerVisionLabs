#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "QGraphicsPixmapItem"
#include "QFileDialog"
#include "QGraphicsScene"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Комбобокс выбора типа обратботки границ изображения
    ui->EdgeWorkTypeComboBox->insertItem(0, "Копирование края");
    ui->EdgeWorkTypeComboBox->insertItem(1, "Снаружи все черное");
    ui->EdgeWorkTypeComboBox->insertItem(2, "Заворачиваение края");
}

MainWindow::~MainWindow()
{
    delete ui;
}

// Загрузить изображение для работы
void MainWindow::on_actionOpenImage_triggered()
{
    QString fileName = QFileDialog::getOpenFileName(this,
                                                    "Open Image",
                                                    nullptr,
                                                    "Image files(*.png *.jpg)");
    QImage img;
    img.load(fileName);

    myImage = new Image(img.height(), img.width());
    myOperation->grayScale(img, *myImage);

    showImage(true);
}

// Фильтр Гаусса
void MainWindow::on_GaussBlurButton_clicked()
{
    myOperation->gaussianBlur(ui->doubleSpinBox->value(),
                                 *myImage,
                                 (EDGEWORKTYPE)ui->EdgeWorkTypeComboBox->currentIndex());

    showImage(false);
}

// Оператор Собеля
void MainWindow::on_SobelButton_clicked()
{
    myOperation->sobel((EDGEWORKTYPE)ui->EdgeWorkTypeComboBox->currentIndex(),
                          *myImage);

    showImage(false);
}

// Вывод изображения на экран
void MainWindow::showImage(bool loadImage)
{
    if (!loadImage)
        ui->graphicsView->scene()->clear();

    QGraphicsScene *scene = new QGraphicsScene();
    QGraphicsPixmapItem* item = new QGraphicsPixmapItem(QPixmap::fromImage(myImage->getImage()));
    scene->addItem(item);
    ui->graphicsView->setScene(scene);
}

// Построение пирамиды Гаусса
void MainWindow::on_pushButton_clicked()
{
    myPyramidImage = myOperation->gaussPyramid(*myImage,
                                               ui->colOctave->value(),
                                               ui->colScales->value(),
                                               ui->doubleSpinBox->value(),
                                               (EDGEWORKTYPE)ui->EdgeWorkTypeComboBox->currentIndex());

    // Сохранение всех изображений полученной пирамиды
    for (int i = 0; i <= ui->colOctave->value(); i++)
    {
        try {
            Image temp = myPyramidImage.getImageInOctaves(i);
            float tempSigma = myPyramidImage.getSigmasInOctaves(i);
            QString fileName = QString("_octave_%1_sigma_%2.png").arg(i).arg(tempSigma);
            temp.getImage().save(fileName);
        }
        catch(...) {}
    }
}

// Алгоритм Моравика
void MainWindow::on_MoravecButton_clicked()
{
    ui->graphicsView->scene()->clear();
    QGraphicsScene *scene = new QGraphicsScene();
    QGraphicsPixmapItem* item = new QGraphicsPixmapItem(QPixmap::fromImage(myImage->getImage()));
    scene->addItem(item);

    // Необходимость использования алгоритма подавления немаксимумов
    bool useNonMax = ui->useNonMaximum->checkState() == Qt::Checked;

    // Собственно поиск интересных точек алгоритмом Моравика
    auto points = myOperation->moravec(*myImage,        // Изображение
                                          1800,         // Пороговое значение
                                          7,            // Высота окна Моравика
                                          7,            // Ширина окна Моравика
                                          useNonMax,    // Флаг ANMS
                                          500);         // Количество точек для ANMS

    // Нарисовать полученные точки на изображении
    QPen pen;
    pen.setColor(Qt::GlobalColor::red);
    for (size_t i = 0; i < points.size(); i++)
    {
        scene->addEllipse(points[i].x(), points[i].y(), 1, 1, pen);
    }
    ui->graphicsView->setScene(scene);
}

// Алгоритм Харрисона
void MainWindow::on_HarrisonButton_clicked()
{
    ui->graphicsView->scene()->clear();
    QGraphicsScene *scene = new QGraphicsScene();

    // Необходимость применения фильтра Гаусса к изображению
    if (ui->useGauss->checkState() == Qt::Checked)
            myOperation->gaussianBlur(ui->sigmaForHarrison->value(),
                                        *myImage,
                                        (EDGEWORKTYPE)ui->EdgeWorkTypeComboBox->currentIndex());

    // Необходимость использования алгоритма подавления немаксимумов
    bool useNonMax = ui->useNonMaximum->checkState() == Qt::Checked;

    // Собственно поиск интересных точек алгоритмом Харриса
    auto points = myOperation->harris(*myImage,              // Изображение
                                      ui->Tvalue->value(),   // Пороговое значение
                                      0.06,                  // Эмпирически определяемая константа [0.04, 0.06]
                                      useNonMax,             // Флаг ANMS
                                      100);                  // Количество точек для ANMS

    QGraphicsPixmapItem* item = new QGraphicsPixmapItem(QPixmap::fromImage(myImage->getImage()));
    scene->addItem(item);

    // Нарисовать точки на изображении
    QPen pen;
    pen.setColor(Qt::GlobalColor::red);
    for (size_t i = 0; i < points.size(); i++)
    {
        scene->addEllipse(points[i].x(), points[i].y(), 1, 1, pen);
    }
    ui->graphicsView->setScene(scene);
}
