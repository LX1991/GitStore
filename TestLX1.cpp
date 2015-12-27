//#include "FileOperation.h"
//#include "Camera.h"
//#include "Model3D.h"
//#include "OcclusionDetectAlgorithms.h"
//
//using namespace std;
////���庯��
//void showDepthImage();
//void acquireDepthImage();
//
////�������
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
////ͼ����
//int Image_num = 0;
////����ɼ���ǰ������ͼ��
//FileCreater *fileCreaterSet1;
//FileCreater *fileCreaterSet2;
////�洢�ɼ������ͼ��
//CvMat * originalDepthData;
//CvMat * currentFrame;
////�洢YAML�ļ�
//FileLoader *myFileLoader;
////�����ģ��
//Camera myCamera;
//Model3D *myModel;
//FileCreater *myFileCreater;
//LJXOcclusionDetectAlgorithms *myAlgorithm;//�ڵ���ⷽ������
//
////��ʼ��opengl����
//void initOpenglConfig()
//{
//	glClearColor(0.0, 0.0, 0.0, 0.0);
//	glEnable(GL_DEPTH_TEST); 	
//	glMatrixMode(GL_PROJECTION); 
//	glLoadIdentity(); 
//	double aspect = myCamera.fov_horizontal/myCamera.fov_vertical; //�Ӿ���Ŀ�߱�
//	//����ͶӰ����
//	gluPerspective(myCamera.fov_horizontal, aspect, myCamera.nearDistance,myCamera.farDistance); 
//	gluLookAt(
//		myCamera.position.x,myCamera.position.y,myCamera.position.z,
//		myCamera.lookAtPoint.x,myCamera.lookAtPoint.y,myCamera.lookAtPoint.z,
//		0,0,1
//		);
//}
//
////ѭ����ʾ
//void Timer1(int iUnused)
//{
//	glutPostRedisplay();
//	glutTimerFunc(30, Timer1, 0);
//}
//
////��ʾģ��
//void displayModel()
//{
//	//����Դ�
//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//
//	//�ɼ���ͼ����
//	Image_num ++ ;
//
//	glPushMatrix();
//	//���������˶���ƽ��
//	glTranslatef(Tranlation_x, Tranlation_y, Tranlation_z);
//	Tranlation_x += 3;
//	//Tranlation_x += (rand() % 6);
//	////���������˶�����ת
//	//glRotatef(Rotation_angle, Rotation_x, Rotation_y, Rotation_z);
//	//Rotation_angle += (rand()/1000);
//	//Rotation_x += 3;
//	//Rotation_y += 3;
//
//
//	//����ģ�͵���
//	for (int i = 0;i < myModel->peaks_1.size();i++)
//	{		
//		//��ȡ������������ֵ
//		int pointIndex1 = myModel->peaks_1.at(i);
//		int pointIndex2 = myModel->peaks_2.at(i);
//		int pointIndex3 = myModel->peaks_3.at(i);
//		
//		//����������
//		glBegin(GL_TRIANGLES);
//		//��ȡ��������
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
//		//���㵱ǰ����������ֵ���Ӷ�ȷ������ʾ����ɫ
//		//��ɫֵ��Χ0-1
//		GLdouble centeralPointX = (point1X+point2X+point3X)/3.0f;
//		GLdouble centeralPointY = (point1Y+point2Y+point3Y)/3.0f;
//		GLdouble centeralPointZ = (point1Z+point2Z+point3Z)/3.0f;
//
//		CvPoint3D64f vectorPointToCameraPos = cvPoint3D64f(
//			centeralPointX - myCamera.position.x,
//			centeralPointY - myCamera.position.y,
//			centeralPointZ - myCamera.position.z
//			);
//		//���㵱ǰ������ֵ�����ֵDEPTH=vectorPointToCameraPos*COS<myCamera.oriention,vectorPointToCameraPos>
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
//	//����������
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
////��ʾ���ͼ��,�������ͼ�����Ԥ����
//void showDepthImage()
//{
//	cvNamedWindow("Depth_Image",1);
//	originalDepthData = cvCreateMat(winSizeWidth,winSizeHeight,CV_32FC1);
//	CvMat * depthMat = cvCreateMat(winSizeWidth,winSizeHeight,CV_32FC1);
//	float * depth=new float[winSizeWidth*winSizeHeight];
//	glReadBuffer(GL_DEPTH);
//	//��ȡ���,�浽depthָ��ָ�������
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
//			//opencv��opengl����ϵ�����෴
//			cvmSet(originalDepthData,winSizeHeight-1-row,col,tempDepth);
//			cvmSet(depthMat,winSizeHeight-1-row,col,tempDepthRange);
//		}		
//	delete [] depth;
//	depth=NULL;
//
//	//����������YAML��ʽ�ļ�
//	myFileCreater = new FileCreater;
//	GLint viewport[4];
//	GLdouble modelview[16];
//	GLdouble projection[16];
//
//	glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
//	glGetDoublev( GL_PROJECTION_MATRIX, projection );
//	glGetIntegerv( GL_VIEWPORT, viewport );
//
//	//�������ͼ��õ���ά����,��ͶӰ�任
//	myFileCreater->createYAMLFileByOpenGL(depthMat, originalDepthData, modelview,projection,viewport);
//       
//	cvShowImage("Depth_Image",originalDepthData);
//
//	acquireDepthImage();
//}
//
////�ɼ�ͼ����̣���ʵ���㷨
//void acquireDepthImage()
//{
//	//�ɼ�ͼ�����
//	//��һ��ͼ��
//	if(Image_num == 1)
//	{
//		cvNamedWindow("Image_1",1);
//		*fileCreaterSet1 = *myFileCreater;
//		//originalDepthData�ɼ����������ͼ��ĻҶ�ͼ��ʾ
//		cvShowImage("Image_1", originalDepthData);
//		//��ʼ�����ͼ����󼰵����������
//		myFileLoader->depthMat = cvCreateMat(myFileCreater->myYAMLFile->depthMat->rows,myFileCreater->myYAMLFile->depthMat->cols,CV_32FC1);
//		myFileLoader->model_loaded->coordinateMatX = cvCreateMat(myFileCreater->myYAMLFile->coordinateMatX->rows,myFileCreater->myYAMLFile->coordinateMatX->cols,CV_32FC1);
//		myFileLoader->model_loaded->coordinateMatY = cvCreateMat(myFileCreater->myYAMLFile->coordinateMatY->rows,myFileCreater->myYAMLFile->coordinateMatY->cols,CV_32FC1);
//		myFileLoader->model_loaded->coordinateMatZ = cvCreateMat(myFileCreater->myYAMLFile->coordinateMatZ->rows,myFileCreater->myYAMLFile->coordinateMatZ->cols,CV_32FC1);
//		cvCopy(myFileCreater->myYAMLFile->depthMat, myFileLoader->depthMat);
//		cvCopy(myFileCreater->myYAMLFile->coordinateMatX, myFileLoader->model_loaded->coordinateMatX);
//		cvCopy(myFileCreater->myYAMLFile->coordinateMatY, myFileLoader->model_loaded->coordinateMatY);
//		cvCopy(myFileCreater->myYAMLFile->coordinateMatZ, myFileLoader->model_loaded->coordinateMatZ);
//		//Ӧ�������Ȳ�ֵ��ֵ����ڵ�
//		myAlgorithm = new LJXOcclusionDetectAlgorithms(myFileLoader->depthMat,myCamera.oriention,myModel,Feature::YAML);
//		myAlgorithm->setAlgorithmType(LJXOcclusionDetectAlgorithms::HARD_TRHESHOLD_MAXDD);
//		float myHardThresholdForMaxDD = 20.0f;
//		myAlgorithm->setHardThresholdForMaxDD(myHardThresholdForMaxDD);
//		//����ڵ�
//		myAlgorithm->detectOcclusion();
//		//�������ڽӱ߽�
//		myAlgorithm->calculateNeighbourPoints();
//		//system("pause");
//	}
//	else if(Image_num == 30)
//	{
//		cvNamedWindow("Image_20",1);
//		*fileCreaterSet2 = *myFileCreater;
//		//originalDepthData�ɼ����������ͼ��ĻҶ�ͼ��ʾ
//		cvShowImage("Image_20",originalDepthData);
//		//system("pause");
//	}
//}
//
////������
//int main(int argc, char *argv[])
//{
//	//�ɼ���ǰ������ͼ��ı����ַ
//	fileCreaterSet1 = new FileCreater;
//	fileCreaterSet2 = new FileCreater;
//	//��ȡģ���ļ�
//	myModel = new Model3D;	
//	//��ȡYAML�ļ�
//	myFileLoader = new FileLoader(myModel);
//	////����ģ���ļ�.oogl��ʽ
//	//string filename;
//	//cout << "Please input the filename : ";
//	//cin >> filename;
//	//myFileLoader->load3DModel(filename);
//	myFileLoader->load3DModel("bunny.oogl");
//	//��ģ�͵���������Ϊ����ԭ��
//	//��������
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
//	//�ƶ����ĵ�ԭ��
//	for (int i = 0;i < myFileLoader->model_loaded->coordinate_x.size();i++)
//	{
//		myFileLoader->model_loaded->coordinate_x[i]=myFileLoader->model_loaded->coordinate_x[i]-centroid_x;
//		myFileLoader->model_loaded->coordinate_y[i]=myFileLoader->model_loaded->coordinate_y[i]-centroid_y;
//		myFileLoader->model_loaded->coordinate_z[i]=myFileLoader->model_loaded->coordinate_z[i]-centroid_z;
//	}
//	//�趨������ڲ���
//	myCamera.fov_vertical = 60;//��ֱ�ӳ���
//	myCamera.fov_horizontal = 60;//ˮƽ�ӳ���
//	myCamera.nearDistance = 200;//������Ӿ���
//	myCamera.farDistance = 600;//��Զ���Ӿ���
//	//��ʼ������������
//	//ԭʼ�۲�λ��
//	myCamera.position = cvPoint3D64f(0,-1,300);
//	myCamera.lookAtPoint = cvPoint3D64f(0,0,0);
//	myCamera.oriention = cvPoint3D64f(
//		myCamera.lookAtPoint.x - myCamera.position.x,
//		myCamera.lookAtPoint.y - myCamera.position.y,
//		myCamera.lookAtPoint.z - myCamera.position.z
//		);
//	system("pause");
//
//	//OpenGL��ʾ
//	glutInit(&argc, argv);
//	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
//	glutInitWindowPosition(100, 100);
//	glutInitWindowSize(winSizeWidth, winSizeHeight);
//	glutCreateWindow("OpenGL��ʾ����");
//	initOpenglConfig();
//
//	Timer1(0);
//	glutDisplayFunc(&displayModel);
//	
//	glutMainLoop();
//	return 0;
//}