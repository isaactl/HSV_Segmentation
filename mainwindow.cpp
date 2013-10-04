#include <QtGui>
#include <QDrag>
#include "mainwindow.h"
#include "seededhuedlg.h"
#include "SetRegionRGB.h"

//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
pcl::visualization::PCLVisualizer viewer ("test_viz", false);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());       // http://www.pcl-users.org/PointCloud-Ptr-td3559529.html
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Rcloud(new pcl::PointCloud<pcl::PointXYZRGB>());      // save the original data
std::vector<int> pointIdxNKNSearch;
std::vector<float> pointNKNSquaredDistances;
std::vector<int> Cluster;
std::vector<bool> IsChosen;
pcl::PolygonMesh plymesh;

MainWindow::MainWindow()
{
    QTextCodec::setCodecForTr(QTextCodec::codecForName("gb18030"));
    QFont font("微软雅黑",12,QFont::Normal,FALSE);
    this->setFont(font);
    this->setMinimumSize(640,480);
    this->showMinimized();
    widget.setAcceptDrops(false);
    setAcceptDrops(true);
    setCentralWidget(&widget);

    createActions();    // 创建菜单栏响应函数
    createMenus();      // 添加菜单栏
    createStatusBar();  // 添加状态栏
    createToolBar();
    setWindowIcon(QIcon(":/images/pcl.png"));   // 设置窗口图标

    SeedDlg = 0;
    RGBDlg = 0;

    vtkSmartPointer<vtkRenderWindow> renderWindow = viewer.getRenderWindow();
    widget.SetRenderWindow (renderWindow);

    viewer.setupInteractor (widget.GetInteractor (), widget.GetRenderWindow ());
    viewer.getInteractorStyle ()->setKeyboardModifier (pcl::visualization::INTERACTOR_KB_MOD_SHIFT);

    viewer.setBackgroundColor ( 0.10, 0.32, 0.32 );

    //viewer.registerPointPickingCallback( PointPickingEventCallback, (void*)&viewer);
}

MainWindow::~MainWindow()
{
    //delete SeedDlg;
}

void MainWindow::newFile()
{
    plymesh.polygons.clear();
    viewer.removePolygonMesh("sample mesh");
}

void MainWindow::open()
{
    QString fileName = QFileDialog::getOpenFileName(this,
                                                        tr("Open File打开文件"), ".",
                                                        tr("PLY File(*.ply)\n"
                                                           "PCD File(*.pcd)\n"
                                                           "All File(*.*)"));
    if( !fileName.isEmpty() )
    {
        loadFile( fileName );
    }
    else
    {
        QMessageBox::about( NULL, QObject::tr("About Open"),
                            QObject::tr("suffix is empty!"));
    }
}

void MainWindow::dropEvent(QDropEvent *event)
{
    QList<QUrl> urls = event->mimeData()->urls();
    if( urls.isEmpty())
    {
        return;
    }
        QString fileName = urls[0].toLocalFile();
        if( fileName.isEmpty() || ( QFileInfo(fileName).suffix() != "pcd" && QFileInfo(fileName).suffix() != "ply" ) )
            return;
        else
            loadFile(fileName);
}

void MainWindow::dragEnterEvent(QDragEnterEvent *event)
{
    if( event->mimeData()->hasFormat("text/uri-list"))
        event->acceptProposedAction();
}

bool MainWindow::save()
{
    if( curFile.isEmpty() ){
        return saveAs();
    }
    else
    {
        return saveFile( curFile );
    }
}

void MainWindow::setCurrentFile(const QString &fileName)
{
    curFile = fileName;
    setWindowModified(false);

    QString showName = tr("Untitled");

    setWindowTitle(tr("%1[*] - %2").arg(showName)
                                   .arg(tr("3D Viewer")));
}

bool MainWindow::saveAs()
{
    QString fileName = QFileDialog::getSaveFileName( this,
                                                     tr("Save File"), ".",
                                                     tr("PLY File (*.ply)\n"
                                                        "PCD File (*.pcd)"));
    if( fileName.isEmpty() )
        return false;

    return saveFile( fileName );
}

bool MainWindow::saveFile(const QString &fileName)
{
    setCurrentFile( fileName );
    if( QFileInfo(fileName).suffix() == "pcd" )
    {
        pcl::io::savePCDFile( fileName.toStdString(), *cloud );
    }
    else if( QFileInfo(fileName).suffix() == "ply" )
    {
        pcl::toROSMsg(*cloud,plymesh.cloud);
        pcl::io::savePolygonFilePLY( fileName.toStdString(), plymesh );
    }

    statusBar()->showMessage( tr("File Saved"), 2000);
    return true;
}

bool MainWindow::loadFile(const QString &fileName)
{
    statusBar()->showMessage(tr("Loading File..."));

    if( QFileInfo(fileName).suffix() == "pcd" )
    {
        if( pcl::io::loadPCDFile( fileName.toStdString(), *cloud )==-1 )
        {
            QMessageBox::about(this,tr("Warnning"),tr("cannot open this PCD file!"));
            return false;
        }
        initiaPolygon();
    }
    else if( QFileInfo(fileName).suffix() == "ply" )
    {
        if( pcl::io::loadPolygonFilePLY( fileName.toStdString(), plymesh)==-1 )
        {
            QMessageBox::about(this,tr("Warnning"),tr("cannot open this PLY file!"));
            return false;
        }
        if( plymesh.polygons.size()<1 )
            initiaPolygon();
        pcl::fromROSMsg( plymesh.cloud, *cloud);
    }

    Eigen::Vector4f c;
    pcl::compute3DCentroid<pcl::PointXYZRGB>( *cloud, c);       //找质心
    for( size_t i=0;i<cloud->points.size();i++ )
    {
        cloud->points[ i ].x -= c[0];
        cloud->points[ i ].y -= c[1];
        cloud->points[ i ].z -= c[2];
    }
    *Rcloud = *cloud;

    setCurrentFile( fileName );
    // canvert plymesh to pcl::pointcloud
    pcl::toROSMsg(*cloud,plymesh.cloud);

    viewer.removePolygonMesh("sample mesh");
    viewer.addPolygonMesh(plymesh,"sample mesh");

    QString message;
    message = QString::number(cloud->points.size());
    statusBar()->showMessage( tr("读取文件完毕,点云大小为:")+message );
    return true;
}

void MainWindow::RegionGrowing()
{
    if( !RGBDlg )
    {
        RGBDlg = new SetRegionRGB(this);
        connect(RGBDlg,SIGNAL(RegionGrowing(float ,unsigned int,unsigned int ,unsigned int)),
                this,SLOT(RegionGrowing(float ,unsigned int,unsigned int ,unsigned int)));
    }
    RGBDlg->show();
    RGBDlg->raise();
    RGBDlg->activateWindow();
}

void MainWindow::RegionGrowing(float QSDistanceThreshold,unsigned int QSPointColorThreshold,unsigned int QSRegionColorThreshold,unsigned int QSMinClusterSize)
{
//    std::cout<<"Region Groing..."<<std::endl;
//    statusBar()->showMessage(tr("Please wait while the Region growing is processing..."));
//    if( cloud->empty() )
//    {
//        QMessageBox::about( NULL, QObject::tr("About"),
//                        QObject::tr("The Point cloud is empty!"));
//        return;
//    }

//    pcl::fromROSMsg(plymesh.cloud,*cloud);
//    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);

//    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
//    reg.setInputCloud (cloud);
//    reg.setSearchMethod (tree);
//    reg.setDistanceThreshold (QSDistanceThreshold);		// 两同类点的最大距离
//    reg.setPointColorThreshold (QSPointColorThreshold);		// specifies the threshold value for color test between the points
//    reg.setRegionColorThreshold (QSRegionColorThreshold);		// the color threshold value used for testing if regions can be merged.
//    reg.setMinClusterSize (QSMinClusterSize);

//    std::vector <pcl::PointIndices> clusters;
//    reg.extract (clusters);

//    cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
//    cloud = reg.getColoredCloud ();

//    pcl::toROSMsg(*cloud,plymesh.cloud);

//    viewer.removePolygonMesh("sample mesh");
//    viewer.addPolygonMesh(plymesh,"sample mesh");

//    statusBar()->showMessage( tr("Region Grow End"), 2000 );
}

void MainWindow::ResetVector()
{
    Cluster.erase( Cluster.begin(), Cluster.end() );
    *cloud = *Rcloud;
    pcl::toROSMsg(*cloud,plymesh.cloud);
    viewer.removePolygonMesh("sample mesh");
    viewer.addPolygonMesh(plymesh,"sample mesh");
}

void MainWindow::about()
{
    QMessageBox::about(this, tr("About Spreadsheet"),
            tr("<h2>Spreadsheet 1.1</h2>"
               "<p>Copyright &copy; 2008 Software Inc."
               "<p>Spreadsheet is a small application that "
               "demonstrates QAction, QMainWindow, QMenuBar, "
               "QStatusBar, QTableWidget, QToolBar, and many other "
               "Qt classes."));
}

void MainWindow::createActions()
{
    // Create a new file
    newAction = new QAction(tr("&New"), this);
    newAction->setIcon(QIcon(":/images/new.png"));
    newAction->setShortcut(QKeySequence::New);
    newAction->setStatusTip(tr("Create a new frame file"));
    connect(newAction, SIGNAL(triggered()), this, SLOT(newFile()));

    // Open file
    openAction = new QAction(tr("&Open..."), this);
    openAction->setIcon(QIcon(":/images/open.png"));
    openAction->setShortcut(QKeySequence::Open);
    openAction->setStatusTip(tr("Open a file"));
    connect(openAction, SIGNAL(triggered()), this, SLOT(open()));

    // Save
    saveAction = new QAction(tr("&Save"), this);
    saveAction->setIcon(QIcon(":/images/save.png"));
    saveAction->setShortcut(QKeySequence::Save);
    saveAction->setStatusTip(tr("Save the model to disk"));
    connect(saveAction, SIGNAL(triggered()), this, SLOT(save()));

    saveAsAction = new QAction(tr("Save &As..."), this);
    saveAsAction->setStatusTip(tr("Save the model under a new name"));
    connect(saveAsAction, SIGNAL(triggered()), this, SLOT(saveAs()));

    // Exit
    exitAction = new QAction(tr("E&xit"), this);
    exitAction->setShortcut(tr("Ctrl+Q"));
    exitAction->setStatusTip(tr("Exit the application"));
    connect(exitAction, SIGNAL(triggered()), this, SLOT(close()));

    // About
    aboutAction = new QAction(tr("&About"), this);
    aboutAction->setStatusTip(tr("Show the application's About box"));
    connect(aboutAction, SIGNAL(triggered()), this, SLOT(about()));

    // Region_Growing
    Region_Grow = new QAction(tr("Region_Grow"), this);
    Region_Grow->setIcon(QIcon(":/images/region.png"));
    Region_Grow->setStatusTip(tr("start region growing..."));
    connect(Region_Grow, SIGNAL(triggered()), this, SLOT(RegionGrowing()));

    // Retreat
    Retreat = new QAction( tr("Retreat"), this);
    Retreat->setStatusTip(tr("Retreat"));
    Retreat->setIcon(QIcon(":/images/back.png"));
    connect(Retreat, SIGNAL(triggered()), this, SLOT(ResetVector()));

    setpoints = new QAction( tr("set representation to points"),this);
    setpoints->setIcon(QIcon(":/images/points.png"));
    setpoints->setStatusTip(tr("set representation to points"));
    connect( setpoints, SIGNAL(triggered()),this,SLOT(SetToPoints()));


    setwire = new QAction( tr("set representation to wireframe"),this);
    setwire->setIcon(QIcon(":/images/Wire.png"));
    setwire->setStatusTip(tr("set representation to wireframe"));
    connect( setwire, SIGNAL(triggered()),this,SLOT(SetToWire()));

    face = new QAction( tr("set representation to face"),this);
    face->setIcon(QIcon(":/images/face.png"));
    face->setStatusTip(tr("set representation to face"));
    connect( face, SIGNAL(triggered()),this,SLOT(SetToFace()));


    SeededHueAcDlg = new QAction(tr("HSV Para"),this);
    SeededHueAcDlg->setStatusTip(tr("show seeded hue para dialog"));
    connect( SeededHueAcDlg,SIGNAL(triggered()),this,SLOT(SeededHue()));


    RGBToGray = new QAction( tr("GRAY VIEW"),this );
    RGBToGray->setStatusTip(tr("set rgb to gray"));
    connect(RGBToGray,SIGNAL(triggered()),this,SLOT(SetRGBToGray()));
}

void MainWindow::createMenus()
{
    fileMenu = menuBar()->addMenu(tr("&File"));
    fileMenu->addAction(newAction);
    fileMenu->addAction(openAction);
    fileMenu->addAction(saveAction);
    fileMenu->addAction(saveAsAction);
    fileMenu->addAction(exitAction);

    editMenu = menuBar()->addMenu(tr("&Edit"));
    editMenu->addAction(Region_Grow);
    editMenu->addAction(SeededHueAcDlg);

    viewMenu = menuBar()->addMenu(tr("&View"));
    viewMenu->addAction(RGBToGray);
}

void MainWindow::createStatusBar()
{
    locationLabel = new QLabel(" 3D tool ");
    locationLabel->setAlignment(Qt::AlignHCenter);
    locationLabel->setMinimumSize(locationLabel->sizeHint());

    statusBar()->addWidget(locationLabel);
}

void MainWindow::createToolBar()
{
    ToolBar = addToolBar(tr("&Edit"));
    ToolBar->addAction(newAction);
    ToolBar->addAction(openAction);
    ToolBar->addAction(saveAction);
    ToolBar->addAction(Retreat);
    ToolBar->addAction(Region_Grow);
    ToolBar->addAction(setpoints);
    ToolBar->addAction(setwire);
    ToolBar->addAction(face);
}

void MainWindow::SeededHue()
{
    if( !SeedDlg )
    {
        SeedDlg = new SeededHueDlg(this);
        connect(SeedDlg,SIGNAL(SeededHue(float, unsigned int ,float ,unsigned int,bool,bool,float,unsigned int )),
                this,SLOT(SeededHue(float, unsigned int, float ,unsigned int,bool,bool,float,unsigned int )));
    }
    SeedDlg->show();
    SeedDlg->raise();
    SeedDlg->activateWindow();
}

void MainWindow::SeededHue(float HistW,unsigned int SearchDis,float RegionColThr,
                           unsigned int MinCluSize,bool ischeckedc,bool ischeckeds,float Edgep,unsigned int sliderV)
{
    statusBar()->showMessage(tr("Please wait while the seeded hue segmentation is processing..."));
    if( cloud->empty() )
    {
        QMessageBox::about( NULL, QObject::tr("Warning"),
                        QObject::tr("The Point cloud is empty!"));
        return;
    }

    RegionGrowingHSV reg;
    pcl::fromROSMsg(plymesh.cloud,*cloud);
    reg.setInputmesh(plymesh);
    reg.setEdgePoss(Edgep);
    reg.setHistWidth(HistW);
    reg.chufenge(ischeckedc);
    reg.mergeMinSizeRegion(ischeckeds);
    reg.setSearchLength (SearchDis);		// specifies the threshold value for color test between the points
    reg.setRegionColorThreshold (RegionColThr);		// the color threshold value used for testing if regions can be merged.
    reg.setMinClusterSize (MinCluSize);

    std::cout<<"extract ..."<<std::endl;
    reg.extract ();

    cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::cout<<"get colored cloud..."<<std::endl;
    cloud = reg.getColoredCloud ();

    pcl::toROSMsg(*cloud,plymesh.cloud);
    viewer.removePolygonMesh("sample mesh");
    viewer.addPolygonMesh(plymesh,"sample mesh");

    statusBar()->showMessage( tr("Region Grow End"), 2000 );
}

void MainWindow::SetToPoints()
{
    viewer.setRepresentationToPointsForAllActors();
}

void MainWindow::SetToWire()
{
    viewer.setRepresentationToWireframeForAllActors();
}

void MainWindow::SetToFace()
{
    viewer.setRepresentationToSurfaceForAllActors();
}

void MainWindow::SetRGBToGray()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_i(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_r(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloudXYZRGBtoXYZI(*cloud,*cloud_i);

    *cloud_r = *cloud;
    for( size_t a=0;a<cloud->points.size();a++ )
    {
        cloud_r->points[a].r = cloud_i->points[a].intensity;
        cloud_r->points[a].g = cloud_i->points[a].intensity;
        cloud_r->points[a].b = cloud_i->points[a].intensity;
    }

    pcl::toROSMsg(*cloud_r,plymesh.cloud);

    viewer.removePolygonMesh("sample mesh");
    viewer.addPolygonMesh(plymesh,"sample mesh");
}


void MainWindow::initiaPolygon()
{
    plymesh.polygons.clear();
    pcl::Vertices vert;
    vert.vertices.resize(3,0);
    for( size_t i=1;i<cloud->points.size();i++ )
    {
        if( i%2 )
        {
            vert.vertices[0] = i;
            vert.vertices[1] = i-1;
            vert.vertices[2] = i+2;
            if( i+2>=cloud->points.size() )
                vert.vertices[2] = cloud->points.size()-1;
        }
        else
        {
            vert.vertices[0] = i;
            vert.vertices[1] = i+2;
            vert.vertices[2] = i-1;

            if( i+2>=cloud->points.size() )
                vert.vertices[1] = cloud->points.size()-1;
        }
        plymesh.polygons.push_back(vert);
    }
}

