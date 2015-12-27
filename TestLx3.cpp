#include "FileOperation.h"
#include "Camera.h"
#include "Model3D.h"
#include "OcclusionDetectAlgorithms.h"

using namespace std;
//���庯��
void showDepthImage();
void acquireDepthImage();
void constructModel();
//��д����������һ���ɵ�õ�key��һ����key�õ���
int getKeyByPoint(int x, int y, int zoom);
pair<int, int> getPointByKey(int key, int zoom);

//�������
double centroid_x = 0;
double centroid_y = 0;
double centroid_z = 0;
int winSizeWidth = 400;
int winSizeHeight = 400;
float Tranlation_x=0;
float Tranlation_y=0;
float Tranlation_z=0;
float Rotation_angle=0;
float Rotation_x=0;
float Rotation_y=0;
float Rotation_z=0;
//ͼ����
int Image_num = 0;
//�����ڵ�������
int Region_num = 0;
//����ȫ��Сƽ������������
double sumArea = 0;
//����ɼ���ǰ������ͼ��
FileCreater *fileCreaterSet1;
FileCreater *fileCreaterSet2;
//�洢�ɼ������ͼ��
CvMat * originalDepthData;
CvMat * depthMat;
CvMat * currentFrame;
//�洢ͼ�����ʵ���������
CvMat *featureValMatOfMeanCurvature;
CvMat *featureValMatOfGaussianCurvature;
//�洢YAML�ļ�
FileLoader *myFileLoader;
//�����ģ��
Camera myCamera;
Model3D *myModel;
FileCreater *myFileCreater;
//�ڵ���ⷽ������
LJXOcclusionDetectAlgorithms *myAlgorithm;
//������Ĥ����
CvMat *maskMatrix;
//����һ��vector�����洢���ڵ�����
vector<pair<int, int>> closePointPixel;  
//�洢�˵�����
vector<pair<int, int>> endPoint;
//�洢ѡ���Ķ˵�����
vector<pair<int, int>> usedEndPoint;
//����һ���������洢��������ͬһ��������ڵ��߽��
vector<pair<pair<int, int>, int>> occlusionList;
//����һ���������洢��������ͬһ����������ٽӱ߽�㣬���ڵ��߽�һһ��Ӧ
vector<pair<pair<int, int>, int>> neighbourList;
//����Сƽ��ṹ,Сƽ��ṹΪ������
typedef struct Patch{
	int patchNum;
	double patchArea;
	CvPoint3D64f occlusionPoint1;
	CvPoint3D64f occlusionPoint2;
	CvPoint3D64f neighbourPoint1;
	CvPoint3D64f neighbourPoint2;
	CvPoint3D64f patchNormal;
	CvPoint3D64f patchCenter;
}Patch;
//����һ���������洢�ڵ�Сƽ��
vector<pair<Patch, int>> occlusionPatchList;


//������keyֵ
int getKeyByPoint(int x, int y, int zoom)
{
	return x * zoom + y;
}

//��keyֵ�����
pair<int, int> getPointByKey(int key, int zoom)
{
	int x = key / zoom;
	int y = key - x * zoom;
	return make_pair(x, y);
}

//��ʼ��opengl����
void initOpenglConfig()
{
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glEnable(GL_DEPTH_TEST); 	
	glMatrixMode(GL_PROJECTION); 
	glLoadIdentity(); 
	double aspect = myCamera.fov_horizontal/myCamera.fov_vertical; //�Ӿ���Ŀ�߱�
	//����ͶӰ����
	gluPerspective(myCamera.fov_horizontal, aspect, myCamera.nearDistance,myCamera.farDistance); 
	gluLookAt(
		myCamera.position.x,myCamera.position.y,myCamera.position.z,
		myCamera.lookAtPoint.x,myCamera.lookAtPoint.y,myCamera.lookAtPoint.z,
		0,0,1
		);
}

//ѭ����ʾ
void Timer1(int iUnused)
{
	glutPostRedisplay();
	glutTimerFunc(30, Timer1, 0);
}

//��ʾģ��
void displayModel()
{
	//����Դ�
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//�ɼ���ͼ����
	Image_num ++ ;

	glPushMatrix();
	//���������˶���ƽ��
	glTranslatef(Tranlation_x, Tranlation_y, Tranlation_z);
	Tranlation_x += 3;
	//Tranlation_x += (rand() % 6);
	////���������˶�����ת
	//glRotatef(Rotation_angle, Rotation_x, Rotation_y, Rotation_z);
	//Rotation_angle += (rand()/1000);
	//Rotation_x += 3;
	//Rotation_y += 3;


	//����ģ�͵���
	for (int i = 0;i < myModel->peaks_1.size();i++)
	{		
		//��ȡ������������ֵ
		int pointIndex1 = myModel->peaks_1.at(i);
		int pointIndex2 = myModel->peaks_2.at(i);
		int pointIndex3 = myModel->peaks_3.at(i);
		
		//����������
		glBegin(GL_TRIANGLES);
		//��ȡ��������
		//Point1
		GLdouble point1X = myModel->coordinate_x.at(pointIndex1);
		GLdouble point1Y = myModel->coordinate_y.at(pointIndex1);
		GLdouble point1Z = myModel->coordinate_z.at(pointIndex1);
		//Point2
		GLdouble point2X = myModel->coordinate_x.at(pointIndex2);
		GLdouble point2Y = myModel->coordinate_y.at(pointIndex2);
		GLdouble point2Z = myModel->coordinate_z.at(pointIndex2);
		//Point3
		GLdouble point3X = myModel->coordinate_x.at(pointIndex3);
		GLdouble point3Y = myModel->coordinate_y.at(pointIndex3);
		GLdouble point3Z = myModel->coordinate_z.at(pointIndex3);
		//���㵱ǰ����������ֵ���Ӷ�ȷ������ʾ����ɫ
		//��ɫֵ��Χ0-1
		GLdouble centeralPointX = (point1X+point2X+point3X)/3.0f;
		GLdouble centeralPointY = (point1Y+point2Y+point3Y)/3.0f;
		GLdouble centeralPointZ = (point1Z+point2Z+point3Z)/3.0f;

		CvPoint3D64f vectorPointToCameraPos = cvPoint3D64f(
			centeralPointX - myCamera.position.x,
			centeralPointY - myCamera.position.y,
			centeralPointZ - myCamera.position.z
			);
		//���㵱ǰ������ֵ�����ֵDEPTH=vectorPointToCameraPos*COS<myCamera.oriention,vectorPointToCameraPos>
		float pointDepth = (
			myCamera.oriention.x*vectorPointToCameraPos.x+
			myCamera.oriention.y*vectorPointToCameraPos.y+
			myCamera.oriention.z*vectorPointToCameraPos.z
			)
			/
			cvSqrt(
			(float)myCamera.oriention.x*(float)myCamera.oriention.x+
			(float)myCamera.oriention.y*(float)myCamera.oriention.y+
			(float)myCamera.oriention.z*(float)myCamera.oriention.z
			);
		pointDepth = pointDepth>=0?pointDepth:0;
		GLfloat color = pointDepth/myCamera.farDistance;
		glColor3f(color, color, color); 
		glVertex3f(point1X, point1Y, point1Z);
		glVertex3f(point2X, point2Y, point2Z);
		glVertex3f(point3X, point3Y, point3Z);
		glEnd();
	}

	showDepthImage();
	glPopMatrix();
	
	//����������
	{
		glBegin(GL_LINES);
		glColor3f(0.f, 1.f, 0.f); //R G B
		glVertex3f(0, 0,0);
		glVertex3f(500,0, 0);
		glEnd();

		glBegin(GL_LINES);
		glColor3f(1.f, 0.f, 0.f); //R G B
		glVertex3f(0, 0,0);
		glVertex3f(0,500, 0);
		glEnd();

		glBegin(GL_LINES);
		glColor3f(0.f, 0.f, 1.f); //R G B
		glVertex3f(0, 0,0);
		glVertex3f(0,0, 500);
		glEnd();
	}
	////��ʾ������
	//float patchNormalLength = 50.f;
	//vector<pair<Patch, int>>::iterator iterPatch = occlusionPatchList.begin();
	//for(; iterPatch != occlusionPatchList.end(); iterPatch ++)
	//{
	//	Patch occlusionPatch = iterPatch->first;
	//	//CvPoint3D64f endVectorPoint = occlusionPatch.patchCenter + occlusionPatch.patchNormal;
	//	//��ʼ��
	//	GLdouble occlusionBeginPoint_x = occlusionPatch.patchCenter.x;
	//	GLdouble occlusionBeginPoint_y = occlusionPatch.patchCenter.y;
	//	GLdouble occlusionBeginPoint_z = occlusionPatch.patchCenter.z;
	//	//�÷���������ͳһ��
	//	float tempZoom = cvSqrt(
	//			(patchNormalLength * patchNormalLength)/
	//			(occlusionPatch.patchNormal.x * occlusionPatch.patchNormal.x+
	//			occlusionPatch.patchNormal.y * occlusionPatch.patchNormal.y+
	//			occlusionPatch.patchNormal.z * occlusionPatch.patchNormal.z)
	//			);
	//	//�յ�
	//	GLdouble occlusionEndPoint_x = occlusionPatch.patchCenter.x + occlusionPatch.patchNormal.x * tempZoom;
	//	GLdouble occlusionEndPoint_y = occlusionPatch.patchCenter.y + occlusionPatch.patchNormal.y * tempZoom;
	//	GLdouble occlusionEndPoint_z = occlusionPatch.patchCenter.z + occlusionPatch.patchNormal.z * tempZoom;

	//	//���Ʒ�����
	//	glBegin(GL_LINES);
	//		glColor3f(0.f, 1.f, 1.f);
	//		glVertex3f(occlusionBeginPoint_x, occlusionBeginPoint_y, occlusionBeginPoint_z);
	//		glVertex3f(occlusionEndPoint_x, occlusionEndPoint_y, occlusionEndPoint_z);
	//	glEnd();
	//}
	glFlush();

	glutSwapBuffers();
}

//��ʾ���ͼ��,�������ͼ�����Ԥ����
void showDepthImage()
{
	cvNamedWindow("Depth_Image",1);
	originalDepthData = cvCreateMat(winSizeWidth,winSizeHeight,CV_32FC1);
	depthMat = cvCreateMat(winSizeWidth,winSizeHeight,CV_32FC1);
	float * depth=new float[winSizeWidth*winSizeHeight];
	glReadBuffer(GL_DEPTH);
	//��ȡ���,�浽depthָ��ָ�������
	glReadPixels(0,0,winSizeWidth,winSizeHeight,GL_DEPTH_COMPONENT,GL_FLOAT,depth);
	for (int row=0;row<winSizeHeight;row++)
		for(int col=0;col<winSizeWidth;col++)
		{
			float tempDepth = depth[row*winSizeHeight+col];
			float tempDepthRange = myCamera.nearDistance+tempDepth*(myCamera.farDistance-myCamera.nearDistance);
			if (tempDepth == 1)
			{
				tempDepth = 0;
				tempDepthRange = 0;
			}
			//opencv��opengl����ϵ�����෴
			cvmSet(originalDepthData,winSizeHeight-1-row,col,tempDepth);
			cvmSet(depthMat,winSizeHeight-1-row,col,tempDepthRange);
		}		
	delete [] depth;
	depth=NULL;

	//����������YAML��ʽ�ļ�
	myFileCreater = new FileCreater;
	GLint viewport[4];
	GLdouble modelview[16];
	GLdouble projection[16];

	glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
	glGetDoublev( GL_PROJECTION_MATRIX, projection );
	glGetIntegerv( GL_VIEWPORT, viewport );

	//�������ͼ��õ���ά����,��ͶӰ�任
	myFileCreater->createYAMLFileByOpenGL(depthMat, originalDepthData, modelview,projection,viewport);
       
	cvShowImage("Depth_Image",originalDepthData);

/////////////////////**********************************************************************************
////////////////////************************�������ʴ���***********************************************
////////////////////***********************************************************************************/

	featureValMatOfMeanCurvature = cvCreateMat(depthMat->rows, depthMat->cols, CV_32FC1);
	featureValMatOfGaussianCurvature = cvCreateMat(originalDepthData->rows, originalDepthData->cols, CV_32FC1);

	cvSetZero(featureValMatOfMeanCurvature);
	cvSetZero(featureValMatOfGaussianCurvature);
	int windowSize=5;
	//�жϵ�ǰ�Ĵ��ڴ�С�Ƿ�Ϊ����
	if ((windowSize -1)%2 != 0)
	{
		cout<<"������������windowSize����������ֵ"<<endl;
		return;
	}

	//����ֵ����
	double m =(windowSize-1)/2;

	double m2 = pow(m,2);
	double m3 = pow(m,3);
	double m4 = pow(m,4);
	double m5 = pow(m,5);
	double P0 = windowSize;	
	double P1 = 2*m3/3+m2+m/3;
	double P2 = 8*m5/45 +
		4*m4/9 +
		2*m3/9 -
		m2/9 -
		m/15;

	CvMat * vector_d0 = cvCreateMat(1,windowSize,CV_32FC1);
	CvMat * vector_d1 = cvCreateMat(1,windowSize,CV_32FC1);
	CvMat * vector_d2 = cvCreateMat(1,windowSize,CV_32FC1);

	double tempVal = m*(m+1)/3;
	double tempVal1 = 1/P0;
	for (int i=0;i<windowSize;i++)
	{
		double tempVal2 = (m - i)/P1;
		double tempVal3 = (pow((m - i),2) - tempVal)/P2;
		cvmSet(vector_d0,0,i,tempVal1);
		cvmSet(vector_d1,0,i,tempVal2);
		cvmSet(vector_d2,0,i,tempVal3);
	}

	CvMat * mat_du = cvCreateMat(windowSize,windowSize,CV_32FC1);
	CvMat * mat_dv = cvCreateMat(windowSize,windowSize,CV_32FC1);
	CvMat * mat_duu = cvCreateMat(windowSize,windowSize,CV_32FC1);
	CvMat * mat_dvv = cvCreateMat(windowSize,windowSize,CV_32FC1);
	CvMat * mat_duv = cvCreateMat(windowSize,windowSize,CV_32FC1);	

	cvGEMM(vector_d0,vector_d1,1,NULL,1,mat_du,CV_GEMM_A_T);
	cvGEMM(vector_d1,vector_d0,1,NULL,1,mat_dv,CV_GEMM_A_T);
	cvGEMM(vector_d0,vector_d2,1,NULL,1,mat_duu,CV_GEMM_A_T);
	cvGEMM(vector_d2,vector_d0,1,NULL,1,mat_dvv,CV_GEMM_A_T);
	cvGEMM(vector_d1,vector_d1,1,NULL,1,mat_duv,CV_GEMM_A_T);

	CvMat * mat_gu = cvCreateMat(originalDepthData->rows,originalDepthData->cols, CV_32FC1);
	CvMat * mat_gv = cvCreateMat(originalDepthData->rows, originalDepthData->cols, CV_32FC1);
	CvMat * mat_guu = cvCreateMat(originalDepthData->rows, originalDepthData->cols, CV_32FC1);
	CvMat * mat_gvv = cvCreateMat(originalDepthData->rows, originalDepthData->cols, CV_32FC1);
	CvMat * mat_guv = cvCreateMat(originalDepthData->rows, originalDepthData->cols, CV_32FC1);

	cvFilter2D(depthMat,mat_gu,mat_du);
	cvFilter2D(depthMat,mat_gv,mat_dv);
	cvFilter2D(depthMat,mat_guu,mat_duu);
	cvFilter2D(depthMat,mat_gvv,mat_dvv);
	cvFilter2D(depthMat,mat_guv,mat_duv);

	int sum1=0,sum2=0,sum3=0;
	for (int row = 0;row<featureValMatOfMeanCurvature->rows;row++)
		for(int col = 0;col<featureValMatOfMeanCurvature->cols;col++)
		{
			double meanCurvatureVal = 0;
			double gaussionCurvatureVal = 0;
			if (cvmGet(depthMat,row,col) != 0)
			{
				double matVal_gu = cvmGet(mat_gu,row,col);
				double matVal_gv = cvmGet(mat_gv,row,col);
				double matVal_guu = cvmGet(mat_guu,row,col);
				double matVal_gvv = cvmGet(mat_gvv,row,col);
				double matVal_guv = cvmGet(mat_guv,row,col);
				double matVal_gu2 = pow(matVal_gu,2);
				double matVal_gv2 = pow(matVal_gv,2);
				double matVal_guv2 = pow(matVal_guv,2);
				double sqrtVal = sqrt(1+matVal_gu2+matVal_gv2);
				double powVal = pow(sqrtVal,3)*2;
				meanCurvatureVal = 
					(
					(1+matVal_gv2)*matVal_guu+
					(1+matVal_gu2)*matVal_gvv-
					2*matVal_gu*matVal_gv*matVal_guv
					)/powVal;
				gaussionCurvatureVal = 
					(matVal_guu*matVal_gvv-matVal_guv2)/
					pow((1+matVal_gu2+matVal_gv2),2);
			}		
			cvmSet(featureValMatOfMeanCurvature,row,col,meanCurvatureVal);
			cvmSet(featureValMatOfGaussianCurvature,row,col,gaussionCurvatureVal);
		}


	acquireDepthImage();
}

//�ɼ�ͼ����̣���ʵ���㷨
void acquireDepthImage()
{
	//�ɼ�ͼ�����
	//��һ��ͼ��
	if(Image_num == 1)
	{
		cvNamedWindow("Image_1",1);
		*fileCreaterSet1 = *myFileCreater;
		//originalDepthData�ɼ����������ͼ��ĻҶ�ͼ��ʾ
		cvShowImage("Image_1", originalDepthData);
		//����ģ��
		constructModel();
	}
	else if(Image_num == 30)
	{
		cvNamedWindow("Image_20",1);
		*fileCreaterSet2 = *myFileCreater;
		//originalDepthData�ɼ����������ͼ��ĻҶ�ͼ��ʾ
		cvShowImage("Image_20",originalDepthData);
		//system("pause");
	}
}

//����ڵ��������ڵ�����ģ��
void constructModel()
{
	//��ʼ�����ͼ����󼰵����������
	myFileLoader->depthMat = cvCreateMat(myFileCreater->myYAMLFile->depthMat->rows,myFileCreater->myYAMLFile->depthMat->cols,CV_32FC1);
	myFileLoader->model_loaded->coordinateMatX = cvCreateMat(myFileCreater->myYAMLFile->coordinateMatX->rows,myFileCreater->myYAMLFile->coordinateMatX->cols,CV_32FC1);
	myFileLoader->model_loaded->coordinateMatY = cvCreateMat(myFileCreater->myYAMLFile->coordinateMatY->rows,myFileCreater->myYAMLFile->coordinateMatY->cols,CV_32FC1);
	myFileLoader->model_loaded->coordinateMatZ = cvCreateMat(myFileCreater->myYAMLFile->coordinateMatZ->rows,myFileCreater->myYAMLFile->coordinateMatZ->cols,CV_32FC1);
	cvCopy(myFileCreater->myYAMLFile->depthMat, myFileLoader->depthMat);
	cvCopy(myFileCreater->myYAMLFile->coordinateMatX, myFileLoader->model_loaded->coordinateMatX);
	cvCopy(myFileCreater->myYAMLFile->coordinateMatY, myFileLoader->model_loaded->coordinateMatY);
	cvCopy(myFileCreater->myYAMLFile->coordinateMatZ, myFileLoader->model_loaded->coordinateMatZ);
	//Ӧ�������Ȳ�ֵ��ֵ����ڵ�
	myAlgorithm = new LJXOcclusionDetectAlgorithms(myFileLoader->depthMat,myCamera.oriention,myModel,Feature::YAML);
	myAlgorithm->setAlgorithmType(LJXOcclusionDetectAlgorithms::HARD_TRHESHOLD_MAXDD);
	float myHardThresholdForMaxDD = 20.0f;
	myAlgorithm->setHardThresholdForMaxDD(myHardThresholdForMaxDD);
	//����ڵ�
	myAlgorithm->detectOcclusion();
	//�������ڽӱ߽�
	myAlgorithm->calculateNeighbourPoints();
	//system("pause");
	//������ݼ�����
	cout<<"�ڵ��߽���������ڽӱ߽��Թ��� : "<<myAlgorithm->occlusionToNeighbourMap.size()<<endl;

	//���ڵ��߽����д���
	//�����ڵ��߽����Ĥ����
	maskMatrix = cvCreateMat(myFileLoader->depthMat->rows, myFileLoader->depthMat->cols, CV_32FC1);
	cvSetZero(maskMatrix);
	map<int, pair<pair<int, int>, pair<int, int>>>::iterator iterOandN = myAlgorithm->occlusionToNeighbourMap.begin();
	for(; iterOandN != myAlgorithm->occlusionToNeighbourMap.end(); iterOandN++)
	{
		pair<int, int> occlusionPoint = iterOandN->second.first;
		cvmSet(maskMatrix, occlusionPoint.first, occlusionPoint.second, 1.f);
	}
	//���ڵ��߽�Ķ˵�
	//����ÿ���ڵ��߽����Χ�ڵ��߽��ĸ���������˵�
	map<int, pair<pair<int, int>, pair<int, int>>>::iterator iter1 = myAlgorithm->occlusionToNeighbourMap.begin();	
	for(; iter1 != myAlgorithm->occlusionToNeighbourMap.end(); iter1++)
	{
		closePointPixel.clear();
		pair<int, int> occlusionPoint = iter1->second.first;
		for(int i = occlusionPoint.first - 1; i < occlusionPoint.first + 2; i++)
			for(int j = occlusionPoint.second - 1; j < occlusionPoint.second + 2; j++)
			{
				float tempDepth = cvmGet(maskMatrix, i, j);
				if (tempDepth == 1.f)
				{
					pair<int, int> closePoint = make_pair(i, j);
					if(occlusionPoint != closePoint)
					{
						closePointPixel.push_back(closePoint);
					}
				}
			}
		//�����Χ��2�����п����Ƕ˵�
		if(closePointPixel.size() == 2)
		{
			int rowDiff = closePointPixel[0].first - closePointPixel[1].first;
			int colDiff = closePointPixel[0].second - closePointPixel[1].second;
			if(1 == abs(rowDiff) + abs(colDiff))
			{
				//CvScalar Blue = cvScalar(1.f, 0.f, 0.f, 0.f);
				//cvSet2D(OcclusionAndNeighbor, occlusionPoint.first, occlusionPoint.second, Blue);
				//����endpoint�У�����δд
				endPoint.push_back(occlusionPoint);
			}
		}
		else if(closePointPixel.size() == 1)
		{
			//CvScalar Blue = cvScalar(1.f, 0.f, 0.f, 0.f);
			//cvSet2D(OcclusionAndNeighbor, occlusionPoint.first, occlusionPoint.second, Blue);
			endPoint.push_back(occlusionPoint);
		}
	}
	//��ʾ�ڵ��߽�����ٽӱ߽�
	CvMat * OcclusionAndNeighbor = cvCreateMat(myFileLoader->depthMat->rows, myFileLoader->depthMat->cols, CV_32FC3);
	map<int, pair<pair<int, int>, pair<int, int>>>::iterator iter2 = myAlgorithm->occlusionToNeighbourMap.begin();
	for(; iter2 != myAlgorithm->occlusionToNeighbourMap.end(); iter2++)
	{
		CvScalar Red = cvScalar(0.f, 0.f, 1.f, 0.f);
		CvScalar Green = cvScalar(0.f, 1.f, 0.f, 0.f);
		CvScalar Blue = cvScalar(1.f, 0.f, 0.f, 0.f);

		pair<int, int> occlusionPoint = iter2->second.first;
		pair<int, int> neighbourPoint = iter2->second.second;
		vector<pair<int, int>>::iterator iterEndpoint = endPoint.begin();
		for(; iterEndpoint != endPoint.end(); iterEndpoint++)
		{
			pair<int, int> Endpoint = *iterEndpoint;
			cvSet2D(OcclusionAndNeighbor, occlusionPoint.first, occlusionPoint.second, Red);
			cvSet2D(OcclusionAndNeighbor, neighbourPoint.first, neighbourPoint.second, Green);
			cvSet2D(OcclusionAndNeighbor, Endpoint.first, Endpoint.second, Blue);
		}
	}
	cvNamedWindow("OcclusionAndNeighbor", 1);
	cvShowImage("OcclusionAndNeighbor", OcclusionAndNeighbor);
	cout<<"endPoint������"<<endPoint.size()<<endl;

	//���ڵ������ն˵���зֿ�
	vector<pair<int, int>>::iterator iterEndpoint = endPoint.begin();
	for(; iterEndpoint != endPoint.end();)
	{
		//��ѡ��Ķ˵����usedEndPoint��
		usedEndPoint.push_back(*iterEndpoint);
		//С������
		Region_num++;
		//ѡ������ͬһ��region���ڵ��߽�
		pair<int, int> presentPoint = *iterEndpoint;
		int presentPointRow = presentPoint.first;
		int presentPointCol = presentPoint.second;
		occlusionList.push_back(make_pair(presentPoint, Region_num));
		//��ֹ���ã���endPoint��ɾ����
		iterEndpoint = endPoint.erase(iterEndpoint);
		while(1)
		{
			//���Ѿ�ȡֵ��λ�õ���Ĥ�����е�ֵ��ֵΪ0.f
			cvmSet(maskMatrix, presentPointRow, presentPointCol, 0.f);
			//������һ���ڵ����λ�ã���2��4��6��8��1��3��7��9˳������
			//2
			if(cvmGet(maskMatrix, presentPointRow - 1, presentPointCol) == 1.f)
			{
				presentPointRow = presentPointRow - 1;
				presentPoint.first = presentPointRow;
				presentPoint.second = presentPointCol;
				occlusionList.push_back(make_pair(presentPoint, Region_num));
				continue;
			}
			//4
			else if(cvmGet(maskMatrix, presentPointRow, presentPointCol - 1) == 1.f)
			{
				presentPointCol = presentPointCol - 1;
				presentPoint.first = presentPointRow;
				presentPoint.second = presentPointCol;
				occlusionList.push_back(make_pair(presentPoint, Region_num));
				continue;
			}
			//6
			else if(cvmGet(maskMatrix, presentPointRow, presentPointCol + 1) == 1.f)
			{
				presentPointCol = presentPointCol + 1;
				presentPoint.first = presentPointRow;
				presentPoint.second = presentPointCol;
				occlusionList.push_back(make_pair(presentPoint, Region_num));
				continue;
			}
			//8
			else if(cvmGet(maskMatrix, presentPointRow + 1, presentPointCol) == 1.f)
			{
				presentPointRow = presentPointRow + 1;
				presentPoint.first = presentPointRow;
				presentPoint.second = presentPointCol;
				occlusionList.push_back(make_pair(presentPoint, Region_num));
				continue;
			}
			//1
			else if(cvmGet(maskMatrix, presentPointRow-1, presentPointCol - 1) == 1.f)
			{
				presentPointRow = presentPointRow - 1;
				presentPointCol = presentPointCol - 1;
				presentPoint.first = presentPointRow;
				presentPoint.second = presentPointCol;
				occlusionList.push_back(make_pair(presentPoint, Region_num));
				continue;
			}
			//3
			else if(cvmGet(maskMatrix, presentPointRow-1, presentPointCol + 1) == 1.f)
			{
				presentPointRow = presentPointRow - 1;
				presentPointCol = presentPointCol + 1;
				presentPoint.first = presentPointRow;
				presentPoint.second = presentPointCol;
				occlusionList.push_back(make_pair(presentPoint, Region_num));
				continue;
			}
			//7
			else if(cvmGet(maskMatrix, presentPointRow + 1, presentPointCol - 1) == 1.f)
			{
				presentPointRow = presentPointRow + 1;
				presentPointCol = presentPointCol - 1;
				presentPoint.first = presentPointRow;
				presentPoint.second = presentPointCol;
				occlusionList.push_back(make_pair(presentPoint, Region_num));
				continue;
			}
			//9
			else if(cvmGet(maskMatrix, presentPointRow + 1, presentPointCol + 1) == 1.f)
			{
				presentPointRow = presentPointRow + 1;
				presentPointCol = presentPointCol + 1;
				presentPoint.first = presentPointRow;
				presentPoint.second = presentPointCol;
				occlusionList.push_back(make_pair(presentPoint, Region_num));
				continue;
			}
			//��һ�����Ƕ˵�,ע�����Ϊ�˵���Ϊ��������ֹ�������������ҵ���һ���Ƕ˵�ĵ㣬
			//���������ͼ���ڵ��ĸ�����û�б��ж�Ϊ�˵㣬������ӦΪ��ĳ���˵㿪ʼ��һ���˵�
			else
			{
				usedEndPoint.push_back(presentPoint);
				vector<pair<int, int>>::iterator iterEndpoint1 = endPoint.begin();
				for(; iterEndpoint1 != endPoint.end();)
				{
					if(presentPoint == *iterEndpoint1)
					{
						iterEndpoint1 = endPoint.erase(iterEndpoint1);
						break;
					}
					else
					{
						iterEndpoint1++;
					}
				}
				break;
			}
		}
	}
	cout<<"�ڵ�����������"<<Region_num<<endl;

	//�����Ӧ�����ٽӱ߽��List
	vector<pair<pair<int, int>, int>>::iterator iterNeighbour = occlusionList.begin();
	for(; iterNeighbour != occlusionList.end(); iterNeighbour++)
	{
		pair<int, int> occlusionPoint = iterNeighbour->first;
		int tempNum = iterNeighbour->second;
		int point_x = occlusionPoint.first;
		int point_y = occlusionPoint.second;
		int KeyZoom = myAlgorithm->get();
		int tempKey = getKeyByPoint(point_x, point_y, KeyZoom);
		map<int, pair<pair<int, int>, pair<int, int>>>::iterator iterFindKey = myAlgorithm->occlusionToNeighbourMap.begin();
		for(; iterFindKey != myAlgorithm->occlusionToNeighbourMap.end(); iterFindKey++)
		{
			int key = iterFindKey->first;
			if(tempKey == key)
			{
				neighbourList.push_back(make_pair(iterFindKey->second.second, tempNum));
			}
		}
	}
	//����ڵ�������ٽӱ߽�����Ŀ
	cout<<"�ڵ��߽����Ŀ��"<<occlusionList.size()<<endl;
	cout<<"���ٽӱ߽����Ŀ��"<<neighbourList.size()<<endl;

	//��ģ�ڵ�ƽ��
	vector<pair<pair<int, int>, int>>::iterator iterA = occlusionList.begin();
	vector<pair<pair<int, int>, int>>::iterator iterB = occlusionList.begin() + 1;
	vector<pair<pair<int, int>, int>>::iterator iterC = neighbourList.begin();
	vector<pair<pair<int, int>, int>>::iterator iterD = neighbourList.begin() + 1;
	for(; iterB != occlusionList.end(); )
	{
		int numA = iterA->second;
		int numB = iterB->second;
		pair<int, int> occlusionPoint1 = iterA->first;
		pair<int, int> occlusionPoint2 = iterB->first;
		pair<int, int> neighbourPoint1 = iterC->first;
		pair<int, int> neighbourPoint2 = iterD->first;
		if(numA == numB)
		{
			Patch newPatch;
			//Сƽ����
			newPatch.patchNum ++ ;
			//�ڵ��߽��1
			newPatch.occlusionPoint1.x = cvmGet(myFileCreater->myYAMLFile->coordinateMatX, occlusionPoint1.first, occlusionPoint1.second);
			newPatch.occlusionPoint1.y = cvmGet(myFileCreater->myYAMLFile->coordinateMatY, occlusionPoint1.first, occlusionPoint1.second);
			newPatch.occlusionPoint1.z = cvmGet(myFileCreater->myYAMLFile->coordinateMatZ, occlusionPoint1.first, occlusionPoint1.second);
			//�ڵ��߽��2
			newPatch.occlusionPoint2.x = cvmGet(myFileCreater->myYAMLFile->coordinateMatX, occlusionPoint2.first, occlusionPoint2.second);
			newPatch.occlusionPoint2.y = cvmGet(myFileCreater->myYAMLFile->coordinateMatY, occlusionPoint2.first, occlusionPoint2.second);
			newPatch.occlusionPoint2.z = cvmGet(myFileCreater->myYAMLFile->coordinateMatZ, occlusionPoint2.first, occlusionPoint2.second);
			//���ٽӱ߽��1
			newPatch.neighbourPoint1.x = cvmGet(myFileCreater->myYAMLFile->coordinateMatX, neighbourPoint1.first, neighbourPoint1.second);
			newPatch.neighbourPoint1.y = cvmGet(myFileCreater->myYAMLFile->coordinateMatY, neighbourPoint1.first, neighbourPoint1.second);
			newPatch.neighbourPoint1.z = cvmGet(myFileCreater->myYAMLFile->coordinateMatZ, neighbourPoint1.first, neighbourPoint1.second);
			//���ٽӱ߽��2
			newPatch.neighbourPoint2.x = cvmGet(myFileCreater->myYAMLFile->coordinateMatX, neighbourPoint2.first, neighbourPoint2.second);
			newPatch.neighbourPoint2.y = cvmGet(myFileCreater->myYAMLFile->coordinateMatY, neighbourPoint2.first, neighbourPoint2.second);
			newPatch.neighbourPoint2.z = cvmGet(myFileCreater->myYAMLFile->coordinateMatZ, neighbourPoint2.first, neighbourPoint2.second);
			//���ڵ��߽��1ָ���ڵ��߽��2
			CvPoint3D64f vector1 = cvPoint3D64f(
				newPatch.occlusionPoint2.x - newPatch.occlusionPoint1.x,
				newPatch.occlusionPoint2.y - newPatch.occlusionPoint1.y,
				newPatch.occlusionPoint2.z - newPatch.occlusionPoint1.z
				);
			//���ڵ��߽��1ָ�����ٽӱ߽��1
			CvPoint3D64f vector2 = cvPoint3D64f(
				newPatch.neighbourPoint1.x - newPatch.occlusionPoint1.x,
				newPatch.neighbourPoint1.y - newPatch.occlusionPoint1.y,
				newPatch.neighbourPoint1.z - newPatch.occlusionPoint1.z
				);
			//�����ٽӱ߽��1ָ�����ٽӱ߽��2
			CvPoint3D64f vector3 = cvPoint3D64f(
				newPatch.neighbourPoint2.x - newPatch.neighbourPoint1.x,
				newPatch.neighbourPoint2.y - newPatch.neighbourPoint1.y,
				newPatch.neighbourPoint2.z - newPatch.neighbourPoint1.z
				);
			//���ڵ��߽��2ָ�����ٽӱ߽��2
			CvPoint3D64f vector4 = cvPoint3D64f(
				newPatch.neighbourPoint2.x - newPatch.occlusionPoint2.x,
				newPatch.neighbourPoint2.y - newPatch.occlusionPoint2.y,
				newPatch.neighbourPoint2.z - newPatch.occlusionPoint2.z
				);
			//���ڵ��߽��1ָ�����ٽӱ߽��2
			CvPoint3D64f vector5 = cvPoint3D64f(
				newPatch.neighbourPoint2.x - newPatch.occlusionPoint1.x,
				newPatch.neighbourPoint2.y - newPatch.occlusionPoint1.y,
				newPatch.neighbourPoint2.z - newPatch.occlusionPoint1.z
				);
			//���ڵ��߽��2ָ�����ٽӱ߽��1
			CvPoint3D64f vector6 = cvPoint3D64f(
				newPatch.neighbourPoint1.x - newPatch.occlusionPoint2.x,
				newPatch.neighbourPoint1.y - newPatch.occlusionPoint2.y,
				newPatch.neighbourPoint1.z - newPatch.occlusionPoint2.z
				);
			//�������ĸ��������ֱ�Ϊ1 * 5, 3 * 5, 2 * 6, 4 * 6,�����������
			double S1 = sqrt(
				pow(vector1.y * vector5.z - vector1.z * vector5.y, 2) +
				pow(vector1.z * vector5.x - vector1.x * vector5.z, 2) +
				pow(vector1.x * vector5.y - vector1.y * vector5.x, 2)
				);
			double S2 = sqrt(
				pow(vector3.y * vector5.z - vector3.z * vector5.y, 2) +
				pow(vector3.z * vector5.x - vector3.x * vector5.z, 2) +
				pow(vector3.x * vector5.y - vector3.y * vector5.x, 2)
				);
			double S3 = sqrt(
				pow(vector2.y * vector6.z - vector2.z * vector6.y, 2) +
				pow(vector2.z * vector6.x - vector2.x * vector6.z, 2) +
				pow(vector2.x * vector6.y - vector2.y * vector6.x, 2)
				);
			double S4 = sqrt(
				pow(vector4.y * vector6.z - vector4.z * vector6.y, 2) +
				pow(vector4.z * vector6.x - vector4.x * vector6.z, 2) +
				pow(vector4.x * vector6.y - vector4.y * vector6.x, 2)
				);
			//���
			newPatch.patchArea = (S1 + S2 + S3 + S4) / 4;
			//����
			newPatch.patchCenter = cvPoint3D64f(
				(newPatch.occlusionPoint1.x + newPatch.occlusionPoint2.x + newPatch.neighbourPoint1.x + newPatch.neighbourPoint2.x) / 4,
				(newPatch.occlusionPoint1.y + newPatch.occlusionPoint2.y + newPatch.neighbourPoint1.y + newPatch.neighbourPoint2.y) / 4,
				(newPatch.occlusionPoint1.z + newPatch.occlusionPoint2.z + newPatch.neighbourPoint1.z + newPatch.neighbourPoint2.z) / 4
				);
			//������,��Ҫ�жϷ���,��֤����������ָ���ⲿ
			CvPoint2D64f vectorOtoO = cvPoint2D64f(occlusionPoint2.second - occlusionPoint1.second, occlusionPoint1.first - occlusionPoint2.first);
			CvPoint2D64f vectorOtoN = cvPoint2D64f(neighbourPoint1.first - occlusionPoint1.first, neighbourPoint1.second - occlusionPoint1.second);
			int dotProduct = (vectorOtoO.x * vectorOtoN.x + vectorOtoO.y * vectorOtoN.y);
			if(dotProduct > 0)
			{
				//���ұߣ�6 * 5
				newPatch.patchNormal.x = (vector6.y * vector5.z - vector6.z * vector5.y);
				newPatch.patchNormal.y = (vector6.z * vector5.x - vector6.x * vector5.z);
				newPatch.patchNormal.z = (vector6.x * vector5.y - vector6.y * vector5.x);
			}
			else
			{
				//����ߣ�5 * 6
				newPatch.patchNormal.x = (vector5.y * vector6.z - vector5.z * vector6.y);
				newPatch.patchNormal.y = (vector5.z * vector6.x - vector5.x * vector6.z);
				newPatch.patchNormal.z = (vector5.x * vector6.y - vector5.y * vector6.x);
			}
			occlusionPatchList.push_back(make_pair(newPatch, numA));
			iterA ++ ;
			iterB ++ ;
			iterC ++ ;
			iterD ++ ;
			sumArea+=newPatch.patchArea;
		}
		else
		{
			iterA ++ ;
			iterB ++ ;
			iterC ++ ;
			iterD ++ ;
		}
	}
	cout<<"�ڵ����������Ϊ��"<<sumArea<<endl;

} 

//������
int main(int argc, char *argv[])
{
	//�ɼ���ǰ������ͼ��ı����ַ
	fileCreaterSet1 = new FileCreater;
	fileCreaterSet2 = new FileCreater;
	//��ȡģ���ļ�
	myModel = new Model3D;	
	//��ȡYAML�ļ�
	myFileLoader = new FileLoader(myModel);
	////����ģ���ļ�.oogl��ʽ
	//string filename;
	//cout << "Please input the filename : ";
	//cin >> filename;
	//myFileLoader->load3DModel(filename);
	myFileLoader->load3DModel("bunny.oogl");
	//��ģ�͵���������Ϊ����ԭ��
	//��������
	for (int i = 0;i < myFileLoader->model_loaded->coordinate_x.size();i++)
	{
		GLdouble point_x = myFileLoader->model_loaded->coordinate_x.at(i);
		GLdouble point_y = myFileLoader->model_loaded->coordinate_y.at(i);
		GLdouble point_z = myFileLoader->model_loaded->coordinate_z.at(i);
		centroid_x = point_x+centroid_x;
		centroid_y = point_y+centroid_y;
		centroid_z = point_z+centroid_z;
	}
	centroid_x=centroid_x/myFileLoader->model_loaded->coordinate_x.size();
	centroid_y=centroid_y/myFileLoader->model_loaded->coordinate_x.size();
	centroid_z=centroid_z/myFileLoader->model_loaded->coordinate_x.size();
	//�ƶ����ĵ�ԭ��
	for (int i = 0;i < myFileLoader->model_loaded->coordinate_x.size();i++)
	{
		myFileLoader->model_loaded->coordinate_x[i]=myFileLoader->model_loaded->coordinate_x[i]-centroid_x;
		myFileLoader->model_loaded->coordinate_y[i]=myFileLoader->model_loaded->coordinate_y[i]-centroid_y;
		myFileLoader->model_loaded->coordinate_z[i]=myFileLoader->model_loaded->coordinate_z[i]-centroid_z;
	}
	//�趨������ڲ���
	myCamera.fov_vertical = 60;//��ֱ�ӳ���
	myCamera.fov_horizontal = 60;//ˮƽ�ӳ���
	myCamera.nearDistance = 200;//������Ӿ���
	myCamera.farDistance = 600;//��Զ���Ӿ���
	//��ʼ������������
	//ԭʼ�۲�λ��
	myCamera.position = cvPoint3D64f(0,-1,300);
	myCamera.lookAtPoint = cvPoint3D64f(0,0,0);
	myCamera.oriention = cvPoint3D64f(
		myCamera.lookAtPoint.x - myCamera.position.x,
		myCamera.lookAtPoint.y - myCamera.position.y,
		myCamera.lookAtPoint.z - myCamera.position.z
		);
	system("pause");

	//OpenGL��ʾ
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
	glutInitWindowPosition(100, 100);
	glutInitWindowSize(winSizeWidth, winSizeHeight);
	glutCreateWindow("OpenGL��ʾ����");
	initOpenglConfig();

	Timer1(0);
	glutDisplayFunc(&displayModel);
	
	glutMainLoop();
	return 0;
}