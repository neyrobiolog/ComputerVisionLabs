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

QImage imageFirst;
QImage imageSecond;

// Открыть первое изображение для сравнения
void MainWindow::on_OpenFirstImage_triggered()
{
    QString path = QFileDialog::getOpenFileName(
                this,tr("Open file"),NULL,"PNG (*.png | *.jpg)");

    imageFirst = QImage(path);
}

// Открыть второе изображение для сравнения
void MainWindow::on_OpenSecondImage_triggered()
{
    QString path = QFileDialog::getOpenFileName(
                this,tr("Open file"),NULL,"PNG (*.png | *.jpg)");

    imageSecond = QImage(path);
}

// Простое сравнение + инвариантность к повороту
void MainWindow::on_simpleCompareAction_triggered()
{
    Matrix m0 = imageToMatrix(imageFirst);
    Matrix m1 = imageToMatrix(imageSecond);
    auto pairs = PointSearcher().findSamePoints(m0, m1);
    //auto pairs = PointMatcher(0.2).match(m0, m1);          // Lr 5

    // Рисуем найденные точки и линии соответствия
    QImage pictures = QImage(m0.width() * 2, m0.height(), QImage::Format_ARGB32);
    for (int i = 0; i < m0.width(); i++)
    {
        for (int j = 0; j < m0.height(); j++)
        {
            pictures.setPixel(i, j, imageFirst.pixel(i, j));
            pictures.setPixel(i + imageSecond.width(), j, imageSecond.pixel(i, j));
        }
    }

    showPictureWithPoints(pictures,pairs);
}

// Поиск блобов
void MainWindow::on_findBlobs_2_triggered()
{
    if (picture != NULL)
    {
        auto space = ScaleSpace(*picture, 5);
        auto blobs = space.computeDiffs().searchBlobs();
        blobs = BlobFilter().filter(blobs, space);

        QImage img = QImage(*image);
        QPainter painter(&img);
        QPen rpen;
        rpen.setWidth(2);
        rpen.setColor(Qt::red);
        painter.setPen(rpen);

        for (Blob each:blobs)
        {
            double k = pow(2, each.octav);
            double radius = each.sigma * k * M_SQRT2;
            painter.drawEllipse(QPointF(each.x * k, each.y * k), radius, radius);
        }

        painter.end();

        showImage(img);
    }
}

// Сравнение - инвариантность к вращению, масштабу, афинным преобразованиям
void MainWindow::on_scaleCompareAction_2_triggered()
{
    Matrix m0 = imageToMatrix(imageFirst);
    Matrix m1 = imageToMatrix(imageSecond);
    auto pairs = PointMatcher(0.2).match(m0, m1, true);

    QImage pictures = QImage(m0.width()*2, m0.height(),QImage::Format_ARGB32);
    for (int i = 0; i < m0.width(); i++)
    {
        for (int j = 0; j < m0.height(); j++)
        {
            pictures.setPixel(i, j, imageFirst.pixel(i, j));
            pictures.setPixel(i + imageSecond.width(), j, imageSecond.pixel(i, j));
        }
    }

    showPictureWithPoints(pictures, pairs);
}

// Показать дескрипторы
void MainWindow::on_seeDescriptors_2_triggered()
{
    if(picture!=NULL){
        auto descr = SIDiscrBuilder::build(*picture);
        showPictureWithDescr(*image,descr);
    }
}

// Собственно показ дескрипторов
void MainWindow::showPictureWithDescr(QImage &img, vector<Descriptor> &descr)
{
    QPainter painter(&img);
    QPen pen;
    pen.setWidth(1);
    pen.setColor(Qt::red);
    painter.setPen(pen);

    for(auto &each:descr){
        painter.drawEllipse(QPointF(each.x,each.y), each.rad, each.rad);
        double x2 = each.x + each.rad*sin(each.angle);
        double y2 = each.y + each.rad*cos(each.angle);
        painter.drawLine(each.x,each.y,x2,y2);
    }

    painter.end();

    showImage(img);
}

// Трансформация: RANSAC
void MainWindow::on_actionRANSAC_triggered()
{
    Matrix m0 = imageToMatrix(imageFirst);
    Matrix m1 = imageToMatrix(imageSecond);
    auto pairs = PointMatcher(0.1).match(m0, m1, true);

    vector<pair<Point, Point>> swaped;
    for (auto & pair:pairs)
    {
        Point p1(pair.first.x, pair.first.y, 0);
        Point p2(pair.second.x, pair.second.y, 0);

        swaped.emplace_back(std::pair<Point,Point>(p1, p2));
    }

    auto h = Ransac().searchTransform(swaped);

    QImage result((m0.width() + m1.width()) ,
                  (m0.height() + m1.height()) / 1.5,
                  QImage::Format_RGB32);

    QPainter painter(&result);
    painter.translate(300, 0);
    painter.drawImage(0, 0, imageSecond);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setRenderHint(QPainter::SmoothPixmapTransform);
    QTransform transform(h[0], h[3], h[6],
                         h[1], h[4], h[7],
                         h[2], h[5], h[8]);

    painter.setTransform(transform, true);
    painter.drawImage(0, 0, imageFirst);

    showImage(result);
}

// Трансформация: HOUGH
void MainWindow::on_actionHOUGH_triggered()
{
    // Ищем соответствия
    Matrix m0 = imageToMatrix(imageFirst);
    Matrix m1 = imageToMatrix(imageSecond);
    auto pairs = PointMatcher(0.1).match(m0, m1, true);

    // Hough transform
    if (pairs.size() > 2)
    {
        auto transf = Hough(m0.width(),m0.height(),m1.width(),
                            m1.height(),pairs).computeHaugh();

        QImage result = QImage(imageSecond);
        QPainter painter(&result);
        int size1 = m1.width()*transf.scale;
        int size2 = m1.width()*transf.scale;
        painter.drawEllipse(QPointF(transf.x, transf.y), 10, 10);
        QRect rect = QRect(transf.x - m1.width()*transf.scale / 2,
                           transf.y - m1.height()*transf.scale / 2,
                           size1, size2);
        painter.drawRect(rect);
        showImage(result);
    }

    else
    {
        showImage(imageFirst);
    }

}
