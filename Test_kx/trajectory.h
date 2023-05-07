#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <QMainWindow>
#include <QWidget>

extern double X[2000][2];
extern QVector<double> K;

namespace Ui {
class Trajectory;
}

class Trajectory : public QMainWindow
{
    Q_OBJECT

public:
    explicit Trajectory(QWidget *parent = nullptr);
    ~Trajectory();
    void draw_trajectory();

private:
    Ui::Trajectory *ui;

};

#endif // TRAJECTORY_H
