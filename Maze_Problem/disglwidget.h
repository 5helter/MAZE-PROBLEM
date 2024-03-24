#ifndef DISGLWIDGET_H
#define DISGLWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QObject>
#include <vector>
#include <limits>
#include <QMouseEvent>
#include "reswidget.h"

const int maxInt=std::numeric_limits<int>::max();

class DisGLWidget : public QOpenGLWidget,protected QOpenGLFunctions
{
    Q_OBJECT
public:
    enum AlgorithmType
    {
        ASTAR,
        BFS,
        DFS
    };

    DisGLWidget();
    DisGLWidget(QWidget* parent);
    void setStartPoint(int i,int j);
    void setEndPoint(int i,int j);
    void run(AlgorithmType theType,bool enableanim);
    void reinitial();
    void setaccount(int count);
    void setdensity(int den);
    void setMode(int mode);
    static int HMode;//函数的选择
    //附带结果窗口
    ResWidget reswidget;
protected:
    virtual void initializeGL() override;
    virtual void resizeGL(int w,int h) override;
    virtual void paintGL() override;
    virtual void mousePressEvent(QMouseEvent *event) override;
private:
    //辅助结构Point
    class Point
    {
    public:
        static int account;
        int i,j;
        Point();
        Point(int ii,int jj);//x,y是在MAZE中的索引
        Point LeftNode();
        Point RightNode();
        Point UpNode();
        Point DownNode();
        bool operator ==(const Point& op);
        int CalculateManhattanDistance(const Point& dp) const;
        Point getChildNode(int i);//i==0，1，2，3分别是左右上下节点
    };


    //绘图函数
    int Mode=0;//mode==0是运行模式等于1是编辑模式
    int animMode=0;
    unsigned char* textureData;
    int twidth, theight, nchannels;
    Point OpenToClosedNode;
    Point CalculatedNode;
    void (DisGLWidget::*updateFrame)()=nullptr;//动画函数
    void prepareTexture(char* path,int i);
    void drawCube(const Point& p);
    void drawLines();
    void drawBlocks();
    void drawFloor();
    void draw2dquads(const Point& p);
    void process();
    void sleep(unsigned int msec);
    void updateAnimFrame();



    //迷宫数据
    int account=15;
    int density=30;
    std::vector<std::vector<int>> MAZE;
    //算法数据结构
    std::vector<std::pair<Point,int>> OPEND;//存节点和h值
    std::vector<std::vector<std::pair<Point,int>>> PATH;//存父节点和g值
    std::vector<std::pair<Point,int>> CLOSED;//已经走过的节点和f值,其实可以不存h值
    //算法
    bool ASTARFunc();
    bool BFSfunc();
    bool DFSFunc();

    void resizeDateStruct();//调用此函数前请确保startP和Endp为合理的值,此函数回自动清空MAZE数据,并且包含initialPATH的功能
    void generateMazaData();
    void initialPATH();
    int Pathcount();//结算路径长度



    Point startP;
    Point endP;

    int opencount=0;//计数open表
    int pathcount=0;

};

#endif // DISGLWIDGET_H
