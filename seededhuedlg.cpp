#include <QtGui>
#include "seededhuedlg.h"

SeededHueDlg::SeededHueDlg(QWidget *parent) :
    QDialog(parent)
{
    label1 = new QLabel(tr("分割步长:"));
    HistWith = new QLineEdit;
    label1->setBuddy(HistWith);
    HistWith->setText(tr("2"));

    label2 = new QLabel(tr("搜索距离:"));
    SearchLength = new QLineEdit;
    label2->setBuddy(SearchLength);
    SearchLength->setText(tr("2"));

    label4 = new QLabel(tr("合并阈值:"));
    RegionColorThreshold = new QLineEdit;
    label4->setBuddy(RegionColorThreshold);
    RegionColorThreshold->setText(tr("3"));

    label5 = new QLabel(tr("最小区域大小:"));
    MinClusterSize = new QLineEdit;
    label5->setBuddy(MinClusterSize);
    MinClusterSize->setText(tr("400"));

    checkboxc = new QCheckBox(tr("合并颜色相近的区域"));
    checkboxc->setChecked(true);

    checkboxs = new QCheckBox(tr("合并点云较小的区域"));
    checkboxs->setChecked(true);

    label3 = new QLabel(tr("边缘概率(%)："));
    spinBox = new QSpinBox;
    label3->setBuddy(spinBox);
    spinBox->setRange(1,100);
    spinBox->setValue(45);

    label6 = new QLabel(tr("相对变化率(%):45"));
    slider = new QSlider(Qt::Horizontal);
    label6->setBuddy(slider);
    slider->setRange(0,200);
    slider->setValue(45);
    QObject::connect(slider,SIGNAL(valueChanged(int)),
                     this,SLOT(setValue()));

    okButton = new QPushButton(tr("&Find"));
    okButton->setDefault(true);
    okButton->setEnabled(false);

    closeButton = new QPushButton(tr("Close"));

    connect(okButton, SIGNAL(clicked()),
            this, SLOT(okbuttonClick()));
    connect(closeButton, SIGNAL(clicked()),
            this, SLOT(close()));

    QHBoxLayout *Layout1 = new QHBoxLayout;
    Layout1->addWidget(label1);
    Layout1->addWidget(HistWith);

    QHBoxLayout *Layout2 = new QHBoxLayout;
    Layout2->addWidget(label2);
    Layout2->addWidget(SearchLength);

    QHBoxLayout *Layout3 = new QHBoxLayout;
    Layout3->addWidget(checkboxc);
    Layout3->addWidget(checkboxs);

    QHBoxLayout *Layout4 = new QHBoxLayout;
    Layout4->addWidget(label4);
    Layout4->addWidget(RegionColorThreshold);

    QHBoxLayout *Layout5 = new QHBoxLayout;
    Layout5->addWidget(label5);
    Layout5->addWidget(MinClusterSize);

    QHBoxLayout *Layout6 = new QHBoxLayout;
    Layout6->addWidget(label3);
    Layout6->addWidget(spinBox);

    QHBoxLayout *Layout7 = new QHBoxLayout;
    Layout7->addWidget(label6);
    Layout7->addWidget(slider);


    QVBoxLayout *TopLayout = new QVBoxLayout;
    TopLayout->addLayout(Layout1);
    TopLayout->addLayout(Layout2);
    TopLayout->addLayout(Layout4);
    TopLayout->addLayout(Layout5);
    TopLayout->addLayout(Layout6);
    TopLayout->addLayout(Layout3);
    TopLayout->addLayout(Layout7);

    QHBoxLayout *Layout = new QHBoxLayout;
    Layout->addWidget(okButton);
    Layout->addWidget(closeButton);
    Layout->addStretch();

    QVBoxLayout *mainLayout = new QVBoxLayout;
    mainLayout->addLayout(TopLayout);
    mainLayout->addLayout(Layout);
    setLayout(mainLayout);

    setWindowTitle(tr("Set Hue Para"));
    setFixedHeight(sizeHint().height());
    okButton->setEnabled(true);
}


void SeededHueDlg::okbuttonClick()
{
    if( SearchLength->text().isEmpty() || HistWith->text().isEmpty() ||
        RegionColorThreshold->text().isEmpty() || MinClusterSize->text().isEmpty() )
    {
        QMessageBox::about( NULL, QObject::tr("Warning"),
                        QObject::tr("Please input a number first!"));
        return;
    }
    else
    {
        float HistW = HistWith->text().toFloat();
        unsigned int searchDis = SearchLength->text().toUInt();
        float RThr = RegionColorThreshold->text().toFloat();
        unsigned int CSize = MinClusterSize->text().toUInt();
        float Edge = spinBox->text().toFloat()/100.0f;
        if(searchDis<1)
            return;
        emit SeededHue(HistW,searchDis,RThr,CSize,checkboxc->isChecked(),
                       checkboxs->isChecked(),Edge,slider->value());
    }
}

void SeededHueDlg::setValue()
{
    sliderValue = QString::number(slider->value());
    label6->setText(tr("相对变化率(%)：")+sliderValue);
}
