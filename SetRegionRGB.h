#ifndef SETREGIONRGB_H
#define SETREGIONRGB_H

#include <QDialog>

class QLabel;
class QLineEdit;
class QPushButton;

class SetRegionRGB: public QDialog
{
    Q_OBJECT

public:
    SetRegionRGB(QWidget *parent = 0 );

    float QSDistanceThreshold;
    unsigned int QSPointColorThreshold;
    unsigned int QSRegionColorThreshold;
    unsigned int QSMinClusterSize;

private slots:
    void SetValue();


signals:
    void RegionGrowing(float QSDistanceThreshold,unsigned int QSPointColorThreshold,unsigned int QSRegionColorThreshold,unsigned int QSMinClusterSize);

private:

    QLabel *DistanceThreshold;
    QLabel *PointColorThreshold;
    QLabel *RegionColorThreshold;
    QLabel *MinClusterSize;
    QLineEdit *DistanceThreshold_lineEdit;
    QLineEdit *PointColorThreshold_lineEdit;
    QLineEdit *RegionColorThreshold_lineEdit;
    QLineEdit *MinClusterSize_lineEdit;
    QPushButton *okButton;
    QPushButton *cancelButton;
};

#endif // SETREGIONRGB_H
