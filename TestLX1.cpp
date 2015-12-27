//#include "FileOperation.h"
//#include "Camera.h"
//#include "Model3D.h"
//#include "OcclusionDetectAlgorithms.h"
//
//using namespace std;
////定义函数
//void showDepthImage();
//void acquireDepthImage();
//
////定义变量
//double centroid_x = 0;
//double centroid_y = 0;
//double centroid_z = 0;
//int winSizeWidth = 400;
//int winSizeHeight = 400;
//float Tranlation_x=0;
//float Tranlation_y=0;
//float Tranlation_z=0;
//float Rotation_angle=0;
//float Rotation_x=0;
//float Rotation_y=0;
//float Rotation_z=0;
////图像编号
//int Image_num = 0;
////定义采集的前后两幅图像
//FileCreater *fileCreaterSet1;
//FileCreater *fileCreaterSet2;
////存储采集的深度图像
//CvMat * originalDepthData;
//CvMat * currentFrame;
////存储YAML文件
//FileLoader *myFileLoader;
////摄像机模型
//Camera myCamera;
//Model3D *myModel;
//FileCreater *myFileCreater;
//LJXOcclusionDetectAlgorithms *myAlgorithm;//遮挡检测方法对象
//
////初始化opengl参数
//void initOpenglConfig()
//{
//	glClearColor(0.0, 0.0, 0.0, 0.0);
//	glEnable(GL_DEPTH_TEST); 	
//	glMatrixMode(GL_PROJECTION); 
//	glLoadIdentity(); 
//	double aspect = myCamera.fov_horizontal/myCamera.fov_vertical; //视景体的宽高比
//	//设置投影矩阵
//	gluPerspective(myCamera.fov_horizontal, aspect, myCamera.nearDistance,myCamera.farDistance); 
//	gluLookAt(
//		myCamera.position.x,myCamera.position.y,myCamera.position.z,
//		myCamera.lookAtPoint.x,myCamera.lookAtPoint.y,myCamera.lookAtPoint.z,
//		0,0,1
//		);
//}
//
////循环显示
//void Timer1(int iUnused)
//{
//	glutPostRedisplay();
//	glutTimerFunc(30, Timer1, 0);
//}
//
////显示模型
//void displayModel()
//{
//	//清空显存
//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//
//	//采集的图像编号
//	Image_num ++ ;
//
//	glPushMatrix();
//	//物体自身运动：平移
//	glTranslatef(Tranlation_x, Tranlation_y, Tranlation_z);
//	Tranlation_x += 3;
//	//Tranlation_x += (rand() % 6);
//	////物体自身运动：旋转
//	//glRotatef(Rotation_angle, Rotation_x, Rotation_y, Rotation_z);
//	//Rotation_angle += (rand()/1000);
//	//Rotation_x += 3;
//	//Rotation_y += 3;
//
//
//	//载入模型点云
//	for (int i = 0;i < myModel->peaks_1.size();i++)
//	{		
//		//读取顶点坐标索引值
//		int pointIndex1 = myModel->peaks_1.at(i);
//		int pointIndex2 = myModel->peaks_2.at(i);
//		int pointIndex3 = myModel->peaks_3.at(i);
//		
//		//绘制三角形
//		glBegin(GL_TRIANGLES);
//		//获取三角形面
//		//Point1
//		GLdouble point1X = myModel->coordinate_x.at(pointIndex1);
//		GLdouble point1Y = myModel->coordinate_y.at(pointIndex1);
//		GLdouble point1Z = myModel->coordinate_z.at(pointIndex1);
//		//Point2
//		GLdouble point2X = myModel->coordinate_x.at(pointIndex2);
//		GLdouble point2Y = myModel->coordinate_y.at(pointIndex2);
//		GLdouble point2Z = myModel->coordinate_z.at(pointIndex2);
//		//Point3
//		GLdouble point3X = myModel->coordinate_x.at(pointIndex3);
//		GLdouble point3Y = myModel->coordinate_y.at(pointIndex3);
//		GLdouble point3Z = myModel->coordinate_z.at(pointIndex3);
//		//计算当前三角面的深度值，从而确定其显示的颜色
//		//颜色值范围0-1
//		GLdouble centeralPointX = (point1X+point2X+point3X)/3.0f;
//		GLdouble centeralPointY = (point1Y+point2Y+point3Y)/3.0f;
//		GLdouble centeralPointZ = (point1Z+point2Z+point3Z)/3.0f;
//
//		CvPoint3D64f vectorPointToCameraPos = cvPoint3D64f(
//			centeralPointX - myCamera.position.x,
//			centeralPointY - myCamera.position.y,
//			centeralPointZ - myCamera.position.z
//			);
//		//计算当前点的深度值，深度值DEPTH=vectorPointToCameraPos*COS<myCamera.oriention,vectorPointToCameraPos>
//		float pointDepth = (
//			myCamera.oriention.x*vectorPointToCameraPos.x+
//			myCamera.oriention.y*vectorPointToCameraPos.y+
//			myCamera.oriention.z*vectorPointToCameraPos.z
//			)
//			/
//			cvSqrt(
//			(float)myCamera.oriention.x*(float)myCamera.oriention.x+
//			(float)myCamera.oriention.y*(float)myCamera.oriention.y+
//			(float)myCamera.oriention.z*(float)myCamera.oriention.z
//			);
//		pointDepth = pointDepth>=0?pointDepth:0;
//		GLfloat color = pointDepth/myCamera.farDistance;
//		glColor3f(color, color, color); 
//		glVertex3f(point1X, point1Y, point1Z);
//		glVertex3f(point2X, point2Y, point2Z);
//		glVertex3f(point3X, point3Y, point3Z);
//		glEnd();
//	}
//
//	showDepthImage();
//	glPopMatrix();
//	
//	//绘制坐标轴
//	{
//		glBegin(GL_LINES);
//		glColor3f(0.f, 1.f, 0.f); //R G B
//		glVertex3f(0, 0,0);
//		glVertex3f(500,0, 0);
//		glEnd();
//
//		glBegin(GL_LINES);
//		glColor3f(1.f, 0.f, 0.f); //R G B
//		glVertex3f(0, 0,0);
//		glVertex3f(0,500, 0);
//		glEnd();
//
//		glBegin(GL_LINES);
//		glColor3f(0.f, 0.f, 1.f); //R G B
//		glVertex3f(0, 0,0);
//		glVertex3f(0,0, 500);
//		glEnd();
//	}
//
//	glFlush();
//
//	glutSwapBuffers();
//}
//
////显示深度图像,并对深度图像进行预处理
//void showDepthImage()
//{
//	cvNamedWindow("Depth_Image",1);
//	originalDepthData = cvCreateMat(winSizeWidth,winSizeHeight,CV_32FC1);
//	CvMat * depthMat = cvCreateMat(winSizeWidth,winSizeHeight,CV_32FC1);
//	float * depth=new float[winSizeWidth*winSizeHeight];
//	glReadBuffer(GL_DEPTH);
//	//读取深度,存到depth指针指向的数组
//	glReadPixels(0,0,winSizeWidth,winSizeHeight,GL_DEPTH_COMPONENT,GL_FLOAT,depth);
//	for (int row=0;row<winSizeHeight;row++)
//		for(int col=0;col<winSizeWidth;col++)
//		{
//			float tempDepth = depth[row*winSizeHeight+col];
//			float tempDepthRange = myCamera.nearDistance+tempDepth*(myCamera.farDistance-myCamera.nearDistance);
//			if (tempDepth == 1)
//			{
//				tempDepth = 0;
//				tempDepthRange = 0;
//			}
//			//opencv与opengl坐标系上下相反
//			cvmSet(originalDepthData,winSizeHeight-1-row,col,tempDepth);
//			cvmSet(depthMat,winSizeHeight-1-row,col,tempDepthRange);
//		}		
//	delete [] depth;
//	depth=NULL;
//
//	//创建并保存YAML格式文件
//	myFileCreater = new FileCreater;
//	GLint viewport[4];
//	GLdouble modelview[16];
//	GLdouble projection[16];
//
//	glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
//	glGetDoublev( GL_PROJECTION_MATRIX, projection );
//	glGetIntegerv( GL_VIEWPORT, viewport );
//
//	//根据深度图像得到三维坐标,反投影变换
//	myFileCreater->createYAMLFileByOpenGL(depthMat, originalDepthData, modelview,projection,viewport);
//       
//	cvShowImage("Depth_Image",originalDepthData);
//
//	acquireDepthImage();
//}
//
////采集图像过程，并实现算法
//void acquireDepthImage()
//{
//	//采集图像过程
//	//第一幅图像
//	if(Image_num == 1)
//	{
//		cvNamedWindow("Image_1",1);
//		*fileCreaterSet1 = *myFileCreater;
//		//originalDepthData采集到的是深度图像的灰度图显示
//		cvShowImage("Image_1", originalDepthData);
//		//初始化深度图像矩阵及点云坐标矩阵
//		myFileLoader->depthMat = cvCreateMat(myFileCreater->myYAMLFile->depthMat->rows,myFileCreater->myYAMLFile->depthMat->cols,CV_32FC1);
//		myFileLoader->model_loaded->coordinateMatX = cvCreateMat(myFileCreater->myYAMLFile->coordinateMatX->rows,myFileCreater->myYAMLFile->coordinateMatX->cols,CV_32FC1);
//		myFileLoader->model_loaded->coordinateMatY = cvCreateMat(myFileCreater->myYAMLFile->coordinateMatY->rows,myFileCreater->myYAMLFile->coordinateMatY->cols,CV_32FC1);
//		myFileLoader->model_loaded->coordinateMatZ = cvCreateMat(myFileCreater->myYAMLFile->coordinateMatZ->rows,myFileCreater->myYAMLFile->coordinateMatZ->cols,CV_32FC1);
//		cvCopy(myFileCreater->myYAMLFile->depthMat, myFileLoader->depthMat);
//		cvCopy(myFileCreater->myYAMLFile->coordinateMatX, myFileLoader->model_loaded->coordinateMatX);
//		cvCopy(myFileCreater->myYAMLFile->coordinateMatY, myFileLoader->model_loaded->coordinateMatY);
//		cvCopy(myFileCreater->myYAMLFile->coordinateMatZ, myFileLoader->model_loaded->coordinateMatZ);
//		//应用最大深度差值阈值检测遮挡
//		myAlgorithm = new LJXOcclusionDetectAlgorithms(myFileLoader->depthMat,myCamera.oriention,myModel,Feature::YAML);
//		myAlgorithm->setAlgorithmType(LJXOcclusionDetectAlgorithms::HARD_TRHESHOLD_MAXDD);
//		float myHardThresholdForMaxDD = 20.0f;
//		myAlgorithm->setHardThresholdForMaxDD(myHardThresholdForMaxDD);
//		//检测遮挡
//		myAlgorithm->detectOcclusion();
//		//计算下邻接边界
//		myAlgorithm->calculateNeighbourPoints();
//		//system("pause");
//	}
//	else if(Image_num == 30)
//	{
//		cvNamedWindow("Image_20",1);
//		*fileCreaterSet2 = *myFileCreater;
//		//originalDepthData采集到的是深度图像的灰度图显示
//		cvShowImage("Image_20",originalDepthData);
//		//system("pause");
//	}
//}
//
////主函数
//int main(int argc, char *argv[])
//{
//	//采集的前后两幅图像的保存地址
//	fileCreaterSet1 = new FileCreater;
//	fileCreaterSet2 = new FileCreater;
//	//读取模型文件
//	myModel = new Model3D;	
//	//读取YAML文件
//	myFileLoader = new FileLoader(myModel);
//	////输入模型文件.oogl格式
//	//string filename;
//	//cout << "Please input the filename : ";
//	//cin >> filename;
//	//myFileLoader->load3DModel(filename);
//	myFileLoader->load3DModel("bunny.oogl");
//	//将模型的重心设置为坐标原点
//	//计算中心
//	for (int i = 0;i < myFileLoader->model_loaded->coordinate_x.size();i++)
//	{
//		GLdouble point_x = myFileLoader->model_loaded->coordinate_x.at(i);
//		GLdouble point_y = myFileLoader->model_loaded->coordinate_y.at(i);
//		GLdouble point_z = myFileLoader->model_loaded->coordinate_z.at(i);
//		centroid_x = point_x+centroid_x;
//		centroid_y = point_y+centroid_y;
//		centroid_z = point_z+centroid_z;
//	}
//	centroid_x=centroid_x/myFileLoader->model_loaded->coordinate_x.size();
//	centroid_y=centroid_y/myFileLoader->model_loaded->coordinate_x.size();
//	centroid_z=centroid_z/myFileLoader->model_loaded->coordinate_x.size();
//	//移动重心到原点
//	for (int i = 0;i < myFileLoader->model_loaded->coordinate_x.size();i++)
//	{
//		myFileLoader->model_loaded->coordinate_x[i]=myFileLoader->model_loaded->coordinate_x[i]-centroid_x;
//		myFileLoader->model_loaded->coordinate_y[i]=myFileLoader->model_loaded->coordinate_y[i]-centroid_y;
//		myFileLoader->model_loaded->coordinate_z[i]=myFileLoader->model_loaded->coordinate_z[i]-centroid_z;
//	}
//	//设定摄像机内参数
//	myCamera.fov_vertical = 60;//垂直视场角
//	myCamera.fov_horizontal = 60;//水平视场角
//	myCamera.nearDistance = 200;//最近可视距离
//	myCamera.farDistance = 600;//最远可视距离
//	//初始化摄像机外参数
//	//原始观测位置
//	myCamera.position = cvPoint3D64f(0,-1,300);
//	myCamera.lookAtPoint = cvPoint3D64f(0,0,0);
//	myCamera.oriention = cvPoint3D64f(
//		myCamera.lookAtPoint.x - myCamera.position.x,
//		myCamera.lookAtPoint.y - myCamera.position.y,
//		myCamera.lookAtPoint.z - myCamera.position.z
//		);
//	system("pause");
//
//	//OpenGL显示
//	glutInit(&argc, argv);
//	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
//	glutInitWindowPosition(100, 100);
//	glutInitWindowSize(winSizeWidth, winSizeHeight);
//	glutCreateWindow("OpenGL显示窗口");
//	initOpenglConfig();
//
//	Timer1(0);
//	glutDisplayFunc(&displayModel);
//	
//	glutMainLoop();
//	return 0;
//}