#ifndef SEEDEDHUEDLG_H
#define SEEDEDHUEDLG_H

#include <QDialog>

class QLabel;
class QLineEdit;
class QCheckBox;
class QSpinBox;
class QSlider;

class SeededHueDlg : public QDialog
{
    Q_OBJECT

public:
    SeededHueDlg(QWidget *parent = 0);
    
signals:
    void SeededHue(float HistW,unsigned int SearchDis,float RegionColThr,
                   unsigned int MinCluSize,bool ischeckedc,bool ischeckeds,float Edgep,unsigned int sliderV);

private slots:
    void okbuttonClick();
    void setValue();

private:
    QLabel *label1;
    QLabel *label2;
    QLabel *label3;
    QLabel *label4;
    QLabel *label5;
    QLabel *label6;
    QCheckBox *checkboxc;
    QCheckBox *checkboxs;
    QSpinBox *spinBox;
    QSlider *slider;
    QLineEdit *HistWith;
    QLineEdit *SearchLength;
    QLineEdit *RegionColorThreshold;
    QLineEdit *MinClusterSize;
    QPushButton *okButton;
    QPushButton *closeButton;
    QString sliderValue;
};

#endif // SEEDEDHUEDLG_H
