#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "Image.h"
#include "Operation.h"
#include "Pyramid.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    Image* myImage;
    Operation* myOperation;
    Image* newImage;
    Pyramid myPyramidImage;

private slots:
    void on_actionOpenImage_triggered();
    void on_GaussBlurButton_clicked();
    void on_SobelButton_clicked();
    void showImage(bool loadImage);
    void on_pushButton_clicked();
    void on_MoravecButton_clicked();
    void on_HarrisonButton_clicked();
    void on_findBlobs_2_triggered();
    void on_scaleCompareAction_2_triggered();
    void on_seeDescriptors_2_triggered();
    void on_actionRANSAC_triggered();
    void on_actionHOUGH_triggered();
    void on_OpenFirstImage_triggered();
    void on_OpenSecondImage_triggered();
    void on_simpleCompareAction_triggered();
};

#endif // MAINWINDOW_H
