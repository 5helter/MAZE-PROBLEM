#include "disglwidget.h"
#include <QMatrix4x4>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <QCoreApplication>
#include <QTime>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

using std::vector;
using std::cout;
using std::endl;
using std::pair;
using std::make_pair;
DisGLWidget::DisGLWidget()
{

}

DisGLWidget::DisGLWidget(QWidget *parent)
{
    this->setParent(parent);
    reswidget.hide();
}

void DisGLWidget::setStartPoint(int i,int j)
{
    startP.i=i;
    startP.j=j;
}

void DisGLWidget::setEndPoint(int i,int j)
{
    endP.i=i;
    endP.j=j;
}

void DisGLWidget::run(AlgorithmType theType, bool enableanim)
{
    OPEND.clear();
    CLOSED.clear();
    //PATH不能直接clear,要使用initailPATH;
    initialPATH();

    //显示原始图像
    int temp=animMode;
    animMode=0;
    update();
    //想让图像先更新再处理后面需要先处理事件
    process();
    animMode=temp;



    if(enableanim==1)
    {
        updateFrame=updateAnimFrame;
    }
    else
    {
        updateFrame=process;
    }


    switch(theType)
    {
    case BFS:
    {

        if(BFSfunc())
        {
            reswidget.setBFs(Pathcount(),opencount);
        }
        else
        {
            reswidget.setBFs(-1,opencount);//没有找到长度记为-1
        }
        break;
    }
    case ASTAR:
    {
        if(ASTARFunc())
        {
            reswidget.setAstar(Pathcount(),opencount);
        }
        else
        {
            reswidget.setAstar(-1,opencount);//没有找到长度记为-1
        }
        break;
    }
    case DFS:
        if(DFSFunc())
        {
            reswidget.setDFs(Pathcount(),opencount);
        }
        else
        {
            reswidget.setDFs(-1,opencount);//没有找到长度记为-1
        }
        break;
    default:
        break;
    }



}

void DisGLWidget::reinitial()
{
    PATH.clear();
    OPEND.clear();
    CLOSED.clear();

    setStartPoint(account/2,0);
    setEndPoint(account/2,account-1);
    resizeDateStruct();
    //生成地形数据
    generateMazaData();
    updateAnimFrame();
}

void DisGLWidget::setaccount(int count)
{
    this->account=count;
}

void DisGLWidget::setdensity(int den)
{
    this->density=den;
}

void DisGLWidget::setMode(int mode)
{
    Mode=mode;
    updateAnimFrame();
}

bool DisGLWidget::ASTARFunc()
{
    opencount=0;
    //cout<<"("<<endP.i<<","<<endP.j<<")"<<endl;
    //初始化,在运行前已经准备好了MAZE,PATH,CLOSED,OPEND,这里需要初始化
    animMode=1;
    OPEND.push_back(make_pair(startP,startP.CalculateManhattanDistance(endP)+0));

    //执行
    while(!OPEND.empty())
    {
        CLOSED.push_back(OPEND[OPEND.size()-1]);
        Point currentnode=OPEND[OPEND.size()-1].first;//从open表中找出g+f的最小节点
        //cout<<"经过节点"<<"("<<currentnode.i<<","<<currentnode.j<<")"<<endl;
        OPEND.pop_back();

        //动画
        OpenToClosedNode=currentnode;
        CalculatedNode=Point(-1,-1);
        (this->*updateFrame)();


        if(currentnode==endP)
        {
            //无论有没有开启动画显示最后的结果
            animMode=2;
            //动画,animMode=2时仅显示结果
            updateAnimFrame();
            reswidget.setAstar(opencount,-1);
            cout<<"find it"<<endl;
            return 1;
        }
        else//扩展刚刚OPEND表的最小节点
        {
            for(int i=0;i<4;++i)
            {
                Point subPoint=currentnode.getChildNode(i);
                if(subPoint==Point(-1,-1)||MAZE[subPoint.i][subPoint.j]==1)//如果在边界上或者被挡住直接跳过
                {
                    continue;
                }
                else
                {
                    //dg=1
                    int g=PATH[currentnode.i][currentnode.j].second+1;
                    int f=subPoint.CalculateManhattanDistance(endP);
                    //设置标志
                    bool repeated=0;
                    //判断是否在OPEN表中
                    for(auto it=OPEND.begin();it!=OPEND.end();++it)
                    {
                        if(it->first==subPoint)
                        {
                            repeated=1;
                            if(g<PATH[subPoint.i][subPoint.j].second)
                            {
                                //更改父节点和g值
                                PATH[subPoint.i][subPoint.j]=make_pair(currentnode,g);
                                //更改f值
                                it->second=f;
                                break;
                            }
                            else
                                break;
                        }
                    }
                    //判断是否在CLOSED表中
                    for(auto it=CLOSED.begin();it!=CLOSED.end();++it)
                    {
                        if(it->first==subPoint)
                        {
                            repeated=1;
                            if(g<PATH[subPoint.i][subPoint.j].second)
                            {
                                //更改父节点和g值
                                PATH[subPoint.i][subPoint.j]=make_pair(currentnode,g);
                                break;
                            }
                            else
                                break;
                        }
                    }
                    //没有出现过的
                    if(repeated==0)
                    {
                        OPEND.push_back(make_pair(subPoint,f));
                        PATH[subPoint.i][subPoint.j]=make_pair(currentnode,g);

                    }


                    ++opencount;
                    //动画
                    OpenToClosedNode=Point(-1,-1);
                    CalculatedNode=subPoint;
                    (this->*updateFrame)();

                }
            }

        }
        //排序
        std::sort(OPEND.begin(),OPEND.end(),[](const std::pair<Point,int>& a, const std::pair<Point,int>& b){
            return a.second>b.second;
        });
    }
    animMode=0;
    updateAnimFrame();
    reswidget.setAstar(opencount,-1);
    cout<<"not find"<<endl;
    return 0;
}

bool DisGLWidget::BFSfunc()
{
    //初始化,在运行前已经准备好了MAZE,PATH,CLOSED,OPEND,这里需要初始化
    animMode=1;
    OPEND.push_back(make_pair(startP,0));

    opencount=0;
    //执行
    while(!OPEND.empty())
    {
        CLOSED.push_back(OPEND[0]);
        Point currentnode=OPEND[0].first;
        OPEND.erase(OPEND.begin());

        //动画
        OpenToClosedNode=currentnode;
        CalculatedNode=Point(-1,-1);
        (this->*updateFrame)();


        if(currentnode==endP)
        {
            //无论有没有开启动画显示最后的结果
            animMode=2;
            //动画,animMode=2时仅显示结果
            updateAnimFrame();

            cout<<"find it"<<endl;
            return 1;
        }
        else//扩展刚刚OPEND表的最小节点
        {
            for(int i=0;i<4;++i)
            {
                Point subPoint=currentnode.getChildNode(i);
                if(subPoint==Point(-1,-1)||MAZE[subPoint.i][subPoint.j]==1)//如果在边界上或者被挡住直接跳过
                {
                    continue;
                }
                else
                {
                    if(PATH[subPoint.i][subPoint.j].first==Point(-1,-1)&!(subPoint==startP))
                    {
                        OPEND.push_back(make_pair(subPoint,0));
                        PATH[subPoint.i][subPoint.j]=make_pair(currentnode,0);
                    }
                    else
                    {

                    }
                    //动画
                    OpenToClosedNode=Point(-1,-1);
                    CalculatedNode=subPoint;
                    (this->*updateFrame)();
                    ++opencount;
                }
            }

        }
    }
    animMode=0;
    updateAnimFrame();
    cout<<"not find"<<endl;
    return 0;
}

bool DisGLWidget::DFSFunc()
{
//    int limits=1;//深度限制
//    int lastmaxDepth=-1;//上一次能到的最大深度，由于没有上一次记为-1
//    int newmaxDepth=0;//本次到达的最大深度，记为0
//    //执行
//    while(lastmaxDepth<newmaxDepth&&limits<5000000)//当limits增加但是newmaxDepth不变大时结束遍历
//    {
//        OPEND.clear();
//        CLOSED.clear();
//        initialPATH();
//        updateAnimFrame();

//        cout<<"restart"<<endl;

//        animMode=1;
//        //在迭代加深的DFS算法中，OPEND表和CLOSED表的另一个参数用来存储深度
//        OPEND.push_back(make_pair(startP,1));
//        opencount=0;

//        lastmaxDepth=newmaxDepth;
//        while(!OPEND.empty())
//        {
//            //弹出栈顶，这里栈顶是最后一个元素
//            CLOSED.push_back(OPEND[OPEND.size()-1]);
//            Point currentnode=OPEND[OPEND.size()-1].first;
//            int currentDepth=OPEND[OPEND.size()-1].second;
//            OPEND.pop_back();


//            //动画
//            OpenToClosedNode=currentnode;
//            CalculatedNode=Point(-1,-1);
//            (this->*updateFrame)();


//            if(currentnode==endP)
//            {
//                //无论有没有开启动画显示最后的结果
//                animMode=2;
//                //动画,animMode=2时仅显示结果
//                updateAnimFrame();

//                cout<<"find it"<<endl;
//                return 1;
//            }
//            else if(currentDepth+1>limits)//如果子节点超过深度限制，不扩展当前节点的子节点
//            {
//                newmaxDepth=limits;
//                continue;
//            }
//            else//如果子节点未超过深度限制，扩展currentnode
//            {
//                for(int i=0;i<4;++i)
//                {
//                    Point subPoint=currentnode.getChildNode(i);
//                    if(subPoint==Point(-1,-1)||MAZE[subPoint.i][subPoint.j]==1)//如果在边界上或者被挡住直接跳过
//                    {
//                        continue;
//                    }
//                    else
//                    {
//                        if(PATH[subPoint.i][subPoint.j].first==Point(-1,-1)&&!(subPoint==startP))
//                        {
//                            OPEND.push_back(make_pair(subPoint,currentDepth+1));
//                            PATH[subPoint.i][subPoint.j]=make_pair(currentnode,0);
//                        }
//                        else
//                        {

//                        }
//                        //动画
//                        OpenToClosedNode=Point(-1,-1);
//                        CalculatedNode=subPoint;
//                        (this->*updateFrame)();
//                        ++opencount;
//                    }
//                }
//            }
//        }
//        ++limits;

//    }
//    animMode=0;
//    updateAnimFrame();
//    if(limits>=5000000)
//    {
//        cout<<"wrong"<<endl;
//    }
//    cout<<"not find"<<endl;
//    return 0;






    animMode=1;
    OPEND.push_back(make_pair(startP,0));
    opencount=0;
    while(!OPEND.empty())
    {
        //弹出栈顶，这里栈顶是最后一个元素
        CLOSED.push_back(OPEND[OPEND.size()-1]);
        Point currentnode=OPEND[OPEND.size()-1].first;
        OPEND.pop_back();

        //动画
        OpenToClosedNode=currentnode;
        CalculatedNode=Point(-1,-1);
        (this->*updateFrame)();


        if(currentnode==endP)
        {
            //无论有没有开启动画显示最后的结果
            animMode=2;
            //动画,animMode=2时仅显示结果
            updateAnimFrame();

            cout<<"find it"<<endl;
            return 1;
        }
        else//如果子节点未超过深度限制，扩展currentnode
        {
            for(int i=3;i>=0;--i)//深度优先遍历对方向有要求否则效率非常低
            {
                Point subPoint=currentnode.getChildNode(i);
                if(subPoint==Point(-1,-1)||MAZE[subPoint.i][subPoint.j]==1)//如果在边界上或者被挡住直接跳过
                {
                    continue;
                }
                else
                {
                    if(PATH[subPoint.i][subPoint.j].first==Point(-1,-1)&&!(subPoint==startP))
                    {
                        OPEND.push_back(make_pair(subPoint,0));
                        PATH[subPoint.i][subPoint.j]=make_pair(currentnode,0);
                    }
                    else
                    {

                    }
                    //动画
                    OpenToClosedNode=Point(-1,-1);
                    CalculatedNode=subPoint;
                    (this->*updateFrame)();
                    ++opencount;
                }
            }
        }
    }
    updateAnimFrame();
    cout<<"not find"<<endl;
    return 0;

}

void DisGLWidget::initializeGL()
{
    initializeOpenGLFunctions();
    this->setUpdateBehavior(QOpenGLWidget::PartialUpdate);//关闭自动清屏
    QOpenGLFunctions* f=QOpenGLContext::currentContext()->functions();
    f->glClearColor(0.5f,0.5f,0.5f,1.0f);
    f->glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    QMatrix4x4 projectionMatrix;
    projectionMatrix.perspective(90, float(this->width()) / this->height(), 0.1f, 100.0f);
    glLoadMatrixf(projectionMatrix.constData());
    glMatrixMode(GL_MODELVIEW);
    glEnable(GL_DEPTH_TEST);


    setStartPoint(account/2,0);
    setEndPoint(account/2,account-1);
    resizeDateStruct();
    //生成地形数据
    generateMazaData();


    char* filePath1="E:/Qt project/Maze_Problem/awesomeface.png";
    char* filePath2="E:/Qt project/Maze_Problem/wrong.png";
    char* filePath3="E:/Qt project/Maze_Problem/routate.png";
    prepareTexture(filePath1,1);
    prepareTexture(filePath2,2);
    prepareTexture(filePath3,3);



}

void DisGLWidget::resizeGL(int w, int h)
{
    initializeOpenGLFunctions();

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    QMatrix4x4 projectionMatrix;
    projectionMatrix.perspective(90, float(w) / h, 0.1f, 100.0f);
    glLoadMatrixf(projectionMatrix.constData());
    glMatrixMode(GL_MODELVIEW);

    glViewport(0, 0, w, h);

}

void DisGLWidget::paintGL()
{
    if(Mode==0)
    {
        glClearColor(0.5f,0.5f,0.5f,1.0f);
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

        glTranslatef(0.0f,0.0f,-1.5f);
        glRotatef(-30.0,1.0f,0.0f,0.0f);

        glDisable(GL_TEXTURE_2D);
        //地板
        drawFloor();
        //网格线
        drawLines();

        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D,2);
        drawBlocks();

        glBindTexture(GL_TEXTURE_2D,3);
        Point temp=endP;
        while(!(PATH[temp.i][temp.j].first==Point(-1,-1)))
        {
            drawCube(startP);
            ++pathcount;
            drawCube(temp);
            temp=PATH[temp.i][temp.j].first;
        }

        glBindTexture(GL_TEXTURE_2D, 1);//启用texture

        for(auto Element:CLOSED)
        {
            drawCube(Element.first);
        }
        for(auto Element:OPEND)
        {
            drawCube(Element.first);
        }


        glLoadIdentity();
    }
    else
    {
        glClearColor(0.5f,0.5f,0.5f,1.0f);
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glOrtho(-1.0,1.0,-1.0,1.0,-1.0,1.0);

        glDisable(GL_TEXTURE);
        //地板
        drawFloor();
        //网格线
        if(account>250)
            return;
        for(int i=1;i<account;++i)
        {
            if(account<190)
                glLineWidth(3);
            else
                glLineWidth(1);


            glBegin(GL_LINES);
            glColor3f(1.0,1.0,1.0);
            glVertex3f(-1.0f+i*(2.0f/account),-1.0f,0.01f);

            glColor3f(1.0,1.0,1.0);
            glVertex3f(-1.0f+i*(2.0f/account),1.0f,0.01f);

            glColor3f(1.0,1.0,1.0);
            glVertex3f(-1.0f,-1.0f+i*(2.0f/account),0.01f);
            glColor3f(1.0,1.0,1.0);
            glVertex3f(1.0f,-1.0f+i*(2.0f/account),0.01f);

            glEnd();
        }
        //绘制阻塞点
        glEnable(GL_TEXTURE);
        glBindTexture(GL_TEXTURE_2D,2);
        for(int i=0;i<account;++i)
            for(int j=0;j<account;++j)
            {
                if(MAZE[i][j]==1)
                    draw2dquads(Point(i,j));
            }
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);

    }

    return;
}

void DisGLWidget::mousePressEvent(QMouseEvent *event)
{

    if(Mode==1)
    {
        float LOS=1.0f/account;
        int j=event->x()/float(this->width())/LOS;
        int i=event->y()/float(this->height())/LOS;
        i=account-1-i;//反转y轴
        if(MAZE[i][j]==0)
        {
            MAZE[i][j]=1;
        }
        else
        {
            MAZE[i][j]=0;
        }
        updateAnimFrame();
        QOpenGLWidget::mousePressEvent(event);
    }
    else
    {
        QOpenGLWidget::mousePressEvent(event);
    }
}

void DisGLWidget::prepareTexture(char* path,int i)
{
   stbi_set_flip_vertically_on_load(true);
   textureData= stbi_load(path, &twidth, &theight, &nchannels, 0);
   if (textureData)
   {
       std::cout << "nchannels==" << nchannels << "width " << twidth << "height " << theight << std::endl;
   }
   else
   {
       std::cout << "Fail" << std::endl;
   }

   glBindTexture(GL_TEXTURE_2D, i);
   glPixelStorei(GL_UNPACK_ALIGNMENT, i);
   //设置重复模式
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
   //设置过滤方式
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
   glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
   if(nchannels==4)
   {
       glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, twidth, theight, 0, GL_RGBA, GL_UNSIGNED_BYTE,textureData);
   }
   else if(nchannels==3)
   {
       glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, twidth, theight, 0, GL_RGB, GL_UNSIGNED_BYTE,textureData);
   }

   stbi_image_free(textureData);
}

void DisGLWidget::drawCube(const Point& p)
{
   int i=p.i;
   int j=p.j;
   float LOS=2.0f/account;

    //绘制正方形四个面
    glBegin(GL_TRIANGLE_STRIP);

    //第一面
    glTexCoord2f(0.0,0.0);
    glVertex3f(-1.0f+LOS*j,-1.0f+LOS*i,0.0f);

    glTexCoord2f(0.0,1.0);
    glVertex3f(-1.0f+LOS*j,-1.0f+LOS*i,LOS);

    glTexCoord2f(1.0,0.0);
    glVertex3f(-1.0f+LOS*j+LOS,-1.0f+LOS*i,0.0f);

    glTexCoord2f(1.0,1.0);
    glVertex3f(-1.0f+LOS*j+LOS,-1.0f+LOS*i,LOS);

    //第二面
    glTexCoord2f(0.0,0.0);
    glVertex3f(-1.0f+LOS*j+LOS,-1.0f+LOS*i+LOS,0.0f);

    glTexCoord2f(0.0,1.0);
    glVertex3f(-1.0f+LOS*j+LOS,-1.0f+LOS*i+LOS,LOS);

    //第三面
    glTexCoord2f(1.0,0.0);
    glVertex3f(-1.0f+LOS*j,-1.0f+LOS*i+LOS,0.0f);

    glTexCoord2f(1.0,1.0);
    glVertex3f(-1.0f+LOS*j,-1.0f+LOS*i+LOS,LOS);

    //第四面
    glTexCoord2f(0.0,0.0);
    glVertex3f(-1.0f+LOS*j,-1.0f+LOS*i,0.0f);

    glTexCoord2f(0.0,1.0);
    glVertex3f(-1.0f+LOS*j,-1.0f+LOS*i,LOS);

    glEnd();

    //绘制顶面
    glColor3f(1.0,1.0,1.0);
    glBegin(GL_TRIANGLE_STRIP);
    glTexCoord2f(0.0,0.0);
    glVertex3f(-1.0f+LOS*j,-1.0f+LOS*i,LOS);

    glTexCoord2f(0.0,1.0);
    glVertex3f(-1.0f+LOS*j+LOS,-1.0f+LOS*i,LOS);

    glTexCoord2f(1.0,0.0);
    glVertex3f(-1.0f+LOS*j,-1.0f+LOS*i+LOS,LOS);

    glTexCoord2f(1.0,1.0);
    glVertex3f(-1.0f+LOS*j+LOS,-1.0f+LOS*i+LOS,LOS);

    glEnd();
}

void DisGLWidget::drawLines()
{
    if(account>250)
       return;
    for(int i=1;i<account;++i)
    {
       if(account<190)
            glLineWidth(3);
       else
            glLineWidth(1);


        glBegin(GL_LINES);
            glColor3f(1.0,1.0,1.0);
            glVertex3f(-1.0f+i*(2.0f/account),-1.0f,0.01f);

            glColor3f(1.0,1.0,1.0);
            glVertex3f(-1.0f+i*(2.0f/account),1.0f,0.01f);

            glColor3f(1.0,1.0,1.0);
            glVertex3f(-1.0f,-1.0f+i*(2.0f/account),0.0f);
            glColor3f(1.0,1.0,1.0);
            glVertex3f(1.0f,-1.0f+i*(2.0f/account),0.0f);

        glEnd();
    }
}

void DisGLWidget::drawBlocks()
{
    float LOS=2.0f/account;
    for(int i=0;i<account;++i)
    {
        for(int j=0;j<account;++j)
        {
            if(MAZE[i][j]==1)
            {
                //绘制正方形四个面
                glBegin(GL_TRIANGLE_STRIP);

                glTexCoord2f(0.0,0.0);
                glVertex3f(-1.0f+LOS*j,-1.0f+LOS*i,0.0f);

                glTexCoord2f(0.0,1.0);
                glVertex3f(-1.0f+LOS*j,-1.0f+LOS*i,LOS);

                glTexCoord2f(1.0,0.0);
                glVertex3f(-1.0f+LOS*j+LOS,-1.0f+LOS*i,0.0f);

                glTexCoord2f(1.0,1.0);
                glVertex3f(-1.0f+LOS*j+LOS,-1.0f+LOS*i,LOS);

                //第二面
                glTexCoord2f(0.0,0.0);
                glVertex3f(-1.0f+LOS*j+LOS,-1.0f+LOS*i+LOS,0.0f);

                glTexCoord2f(0.0,1.0);
                glVertex3f(-1.0f+LOS*j+LOS,-1.0f+LOS*i+LOS,LOS);

                //第三面
                glTexCoord2f(1.0,0.0);
                glVertex3f(-1.0f+LOS*j,-1.0f+LOS*i+LOS,0.0f);

                glTexCoord2f(1.0,1.0);
                glVertex3f(-1.0f+LOS*j,-1.0f+LOS*i+LOS,LOS);

                //第四面
                glTexCoord2f(0.0,0.0);
                glVertex3f(-1.0f+LOS*j,-1.0f+LOS*i,0.0f);
                glTexCoord2f(0.0,1.0);
                glVertex3f(-1.0f+LOS*j,-1.0f+LOS*i,LOS);

                glEnd();

                //绘制顶面
                glBegin(GL_TRIANGLE_STRIP);
                glTexCoord2f(0.0,0.0);
                glVertex3f(-1.0f+LOS*j,-1.0f+LOS*i,LOS);


                glTexCoord2f(0.0,1.0);
                glVertex3f(-1.0f+LOS*j+LOS,-1.0f+LOS*i,LOS);

                glTexCoord2f(1.0,0.0);
                glVertex3f(-1.0f+LOS*j,-1.0f+LOS*i+LOS,LOS);

                glTexCoord2f(1.0,1.0);
                glVertex3f(-1.0f+LOS*j+LOS,-1.0f+LOS*i+LOS,LOS);

                glEnd();
            }
        }
    }
}

void DisGLWidget::drawFloor()
{
    glBegin(GL_TRIANGLE_STRIP);

    glColor3f(0.0f,1.0f,0.0f);
    glVertex3f(-1.0f,1.0f,0.0f);

    glColor3f(0.0f,1.0f,0.0f);
    glVertex3f(-1.0f,-1.0f,0.0f);

    glColor3f(0.0f,1.0f,0.0f);
    glVertex3f(1.0f,1.0f,0.0f);

    glColor3f(0.0f,1.0f,0.0f);
    glVertex3f(1.0f,-1.0f,0.0f);

    glEnd();
}

void DisGLWidget::draw2dquads(const Point &p)
{
    int i=p.i;
    int j=p.j;
    float LOS=2.0f/account;
    glBegin(GL_TRIANGLE_STRIP);
    glTexCoord2f(0.0,0.0);
    glVertex3f(-1.0f+LOS*j,-1.0f+LOS*i,LOS);

    glTexCoord2f(0.0,1.0);
    glVertex3f(-1.0f+LOS*j+LOS,-1.0f+LOS*i,LOS);

    glTexCoord2f(1.0,0.0);
    glVertex3f(-1.0f+LOS*j,-1.0f+LOS*i+LOS,LOS);

    glTexCoord2f(1.0,1.0);
    glVertex3f(-1.0f+LOS*j+LOS,-1.0f+LOS*i+LOS,LOS);
    glEnd();
}

void DisGLWidget::process()
{
    QCoreApplication::processEvents(QEventLoop::AllEvents,100);
}

void DisGLWidget::sleep(unsigned int msec)
{
    QTime reachTime =QTime::currentTime().addMSecs(msec);
    while(QTime::currentTime()<reachTime)
    {
        QCoreApplication::processEvents(QEventLoop::AllEvents,100);
    }
}

void DisGLWidget::updateAnimFrame()
{
    sleep(46);
    update();
}

void DisGLWidget::resizeDateStruct()
{
    MAZE.clear();
    PATH.clear();


    for(int i=0;i<account;++i)
    {
        vector<int> rowmaze;
        vector<pair<Point,int>> rowpath;
        for(int j=0;j<account;++j)
        {
            rowmaze.push_back(0);
            //负一表示未探索到父节点，g值初始为为最大，除了起始节点外
            rowpath.push_back(make_pair(Point(-1,-1),maxInt));
        }
        MAZE.push_back(rowmaze);
        PATH.push_back(rowpath);
    }

    PATH[startP.i][startP.j].second=0;//起始节点g值为0
    Point::account=this->account;//设置点的范围限制
}

void DisGLWidget::generateMazaData()
{
     //debug
    srand((unsigned int)time(0));
    for(int i=0;i<account;++i)
    {
        for(int j=0;j<account;++j)
        {
            if(rand()%100<density)
            {
                MAZE[i][j]=1;
            }
        }
    }
    MAZE[startP.i][startP.j]=0;
    MAZE[endP.i][endP.j]=0;
}

void DisGLWidget::initialPATH()
{
    for(int i=0;i<account;++i)
    {
        for(int j=0;j<account;++j)
        {
            PATH[i][j]=make_pair(Point(-1,-1),maxInt);
        }
    }

    PATH[startP.i][startP.j].second=0;//起始节点g值为0
}

int DisGLWidget::Pathcount()
{
    pathcount=0;
    Point temp=endP;
    while(!(PATH[temp.i][temp.j].first==Point(-1,-1)))
    {
        ++pathcount;
        temp=PATH[temp.i][temp.j].first;
    }
    return pathcount+1;
}

DisGLWidget::Point::Point()
{
    i=-1;
    j=-1;
}

DisGLWidget::Point::Point(int ii, int jj)
{
    i=ii;
    j=jj;
}

DisGLWidget::Point DisGLWidget::Point::LeftNode()
{
    if(j-1<0)
    {
        return Point(-1,-1);
    }
    else
    {
        return Point(i,j-1);
    }
}

DisGLWidget::Point DisGLWidget::Point::RightNode()
{
    if(j+1>=account)
    {
        return Point(-1,-1);
    }
    else
    {
        return Point(i,j+1);
    }
}

DisGLWidget::Point DisGLWidget::Point::UpNode()
{
    if(i-1<0)
    {
        return Point(-1,-1);
    }
    else
    {
        return Point(i-1,j);
    }
}

DisGLWidget::Point DisGLWidget::Point::DownNode()
{
    if(i+1>=account)
    {
        return Point(-1,-1);
    }
    else
    {
        return Point(i+1,j);
    }
}

bool DisGLWidget::Point::operator ==(const Point &op)
{
    return (i==op.i)&&(j==op.j);
}

int DisGLWidget::Point::CalculateManhattanDistance(const Point &dp) const
{
    if(DisGLWidget::HMode==0)
    {
        int dy=std::abs(i-dp.i);
        int dx=std::abs(j-dp.j);
        return dx+dy;
    }
    else
        return 0;

}

DisGLWidget::Point DisGLWidget::Point::getChildNode(int i)
{
    switch (i) {
    case 0:
        return LeftNode();
        break;
    case 1:
        return RightNode();
        break;
    case 2:
        return UpNode();
        break;
    case 3:
        return DownNode();
        break;
    default:
        break;
    }
}
int DisGLWidget::Point::account=0;
int DisGLWidget::HMode=0;
