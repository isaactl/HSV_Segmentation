#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QVTKWidget.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost\thread\thread.hpp>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl\io\ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>
#include <pcl\io\vtk_lib_io.h>
#include "region_growing_hsv.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

class QAction;
class QLabel;
class SeededHueDlg;
class SetRegionRGB;

class MainWindow : public QMainWindow
{
    Q_OBJECT
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    MainWindow();
    ~MainWindow();

private slots:
    void newFile();
    void open();
    bool save();
    bool saveAs();
    void about();
    void RegionGrowing();
    void ResetVector();
    void SeededHue(float HistW,unsigned int SearchDis,float RegionColThr,
                   unsigned int MinCluSize,bool ischeckedc,bool ischeckeds,float Edgep,unsigned int sliderV);
    void RegionGrowing(float QSDistanceThreshold,unsigned int QSPointColorThreshold,
                       unsigned int QSRegionColorThreshold,unsigned int QSMinClusterSize);
    void SetToPoints();
    void SetToWire();
    void SetToFace();
    void SeededHue();
    void SetRGBToGray();

private:

    void createActions();
    void createMenus();
    void createStatusBar();
    void createToolBar();
    bool saveFile(const QString &fileName);
    void setCurrentFile(const QString &fileName);
    bool loadFile(const QString &fileName);
    void initiaPolygon();
    void dropEvent(QDropEvent *event);
    void dragEnterEvent(QDragEnterEvent *event);

    QVTKWidget widget;

    QString curFile;

    QMenu *fileMenu;
    QMenu *editMenu;
    QMenu *viewMenu;

    SeededHueDlg *SeedDlg;
    SetRegionRGB *RGBDlg;

    QToolBar *ToolBar;
    QAction *newAction;
    QAction *openAction;
    QAction *exitAction;
    QAction *aboutAction;
    QAction *Region_Grow;
    QAction *Retreat;
    QAction *SeededHueAcDlg;
    QAction *setpoints;
    QAction *setwire;
    QAction *face;
    QLabel *locationLabel;
    QAction *saveAction;
    QAction *saveAsAction;
    QAction *RGBToGray;
    QAction *pointEditerAction;
};

#endif
