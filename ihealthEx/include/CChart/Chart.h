/*============================================================================*/
/*                                                                            */
/*                            C O P Y R I G H T                               */
/*                                                                            */
/*                          (C) Copyright 2016 by                             */
/*                              Yang Guojun                                   */
/*                           All Rights Reserved                              */
/*                                                                            */
/*      The author assumes no responsibility for the use or reliability of    */
/*      his software.                                                         */
/*                                                                            */
/*============================================================================*/

/* ############################################################################################################################## */

#ifndef __CHART_H_122333444455555__
#define __CHART_H_122333444455555__

#include <windows.h>
#include <tchar.h>

//���ڶ�̬���ӿ��ʱ��ע�͵�#else�еĵڶ���
//���ھ�̬�����Դ��ʱ��ע�͵�#else�еĵ�һ��
#ifdef CChart_EXPORTS
#	define CChart_API __declspec(dllexport)
#else
#	define CChart_API __declspec(dllimport)
//#	define CChart_API
#endif

// CChart��װ��֧�ֵĻ�ͼ����
enum
{
	kTypeXY,							//0 ����ͼ
	kTypePie,							//1 ��ͼ
	kTypeStem,							//2 ��ͼ
	kTypeOscillo,						//3 ģ��ʾ����ͼ
	kTypeContourLine,					//4 �ȸ���ͼ
	kTypeContourMap,					//5 ��ͼ
	kTypeContour,						//6 �ȸ�����ͼ
	kTypeWaterfall,						//7 �ٲ�ͼ

	kTypeSingleLayerCount,				// ������ͼ��Ŀ

	kTypeSplit = kTypeSingleLayerCount,	//8 ������ͼ
	kTypeShareX,						//9 ����X����ͼ
	kTypeLayered,						//10 �ֲ���ͼ
	
	kType2DCount,						// ��ά��ͼ��Ŀ

	kType3DLine = kType2DCount,			//11 3ά����ͼ
	kType3DSurface,						//12 3ά����ͼ

	kTypeCount							//CChart��֧�ֵ���ͼ������
};

const struct stChartType
{
	int nChartType;
	TCHAR szChartType[24];
}ChartTypes[] =
{
	{kTypeXY			,_T("TypeXY")},
	{kTypePie			,_T("TypePie")},
	{kTypeStem			,_T("TypeStem")},
	{kTypeOscillo		,_T("TypeOscillo")},
	{kTypeContourLine	,_T("TypeContourLine")},
	{kTypeContourMap	,_T("TypeContourMap")},
	{kTypeContour		,_T("TypeContour")},
	{kTypeWaterfall		,_T("TypeWaterfall")},
	{kTypeSplit			,_T("TypeSplit")},
	{kTypeShareX		,_T("TypeShareX")},
	{kTypeLayered		,_T("TypeLayered")},
	{kType3DLine		,_T("Type3DLine")},
	{kType3DSurface		,_T("Type3DSurface")},
};

//typedef double (*FieldFunction)(double, int);

/*

// ���߻��Ʒ�ʽ
enum
{
	kXYPlotScatter = 0,		//0 ɢ��ͼ
	kXYPlotConnect = 1,		//1 ����ͼ
	kXYPlotStepHV = 2,		//2 ̨��ͼ��ˮƽ����ֱ
	kXYPlotStepVH = 3,		//3 ̨��ͼ����ֱ��ˮƽ
	kXYPlotStepHVH = 4,		//4 ̨��ͼˮƽ��ֱˮƽ
	kXYPlotStepVHV = 5,		//5 ̨��ͼ��ֱˮƽ��ֱ
	kXYPlotBezier = 6,		//6 ����������ͼ
	kXYPlotBar = 7,			//7 ÿ����ֻ���ƴ������Ὺʼ�����ݰ�
  
	kPlotTypeCount
};

 // ���߶�ɫģʽ����ɫ�ڵ�����뷽ʽ��AddSegColor��һ�������ĺ��壩
 enum
 {
	kSegColorPointRatio = 0,	//0 ���ݵ�ı���
	kSegColorXVal = 1,			//1 X��ֵ
	kSegColorYVal,				//2 Y��ֵ
  
	kSegColorCount
};

// ���ݱ�ǵ���״
enum
{
	kXYMarkerNone = 0,			//0 �ޱ��
	kXYMarkerCircle = 1,		//1 ԲȦ���
	kXYMarkerSquareUpright = 2,	//2 ��������
	kXYMarkerSquareOblique = 3,	//3 б������
	kXYMarkerTriangleLeft = 4,	//4 ����������
	kXYMarkerTriangleRight = 5,	//5 ����������
	kXYMarkerTriangleUp = 6,	//6 ��������
	kXYMarkerTriangleDown = 7,	//7 ��������
	kXYMarkerX = 8,				//8 бʮ��
	kXYMarkerCross = 9,			//9 ��ʮ��
	kXYMarkerBar = 10,			//10��ɫ��
	kXYMarkerDot = 11,			//11��
		
	kMarkerTypeCount
};

 // �������ķ�ʽ
enum
{
	kDataFillClosed = 0,			//0 ������
	kDataFillFromBottomAxis = 1,	//1 �������
	kDataFillFromTopAxis = 2,		//2 �������
	kDataFillFromLeftAxis = 3,      //3 �������
	kDataFillFromRightAxis = 4,		//4 �������
  
	kDataFillModeCount
};
// ���ݰ���ʾ��ʽ
enum
{
	kDataBarBaseBottom,		//0 �ӵײ���ʼ
	kDataBarBaseTop,		//1 �Ӷ�����ʼ
	kDataBarBaseLeft,		//2 ���󲿿�ʼ
	kDataBarBaseRight,		//3 ���Ҳ���ʼ
		
	kDataBarBaseCount
};

// ������ͼ�ķ���ģʽ
// ���ȡ��ע�ͣ�����ֱ���ô��ţ������ô��룬����2��ʾ��һ�Ҷ���������
enum
{
	kSplitNot=0,		//0 ������
	kSplitNM=1,			//1 ���зָ�
	kSplit3L1R2=2,		//2 ��1��2
	kSplit3L2R1=3,		//3 ��2��1
	kSplit3T1B2=4,		//4 ��1��2
	kSplit3T2B1=5,		//5 ��2��1
		
	kSplitModeCount
};

// ������λ��
// ���ȡ��ע�ͣ�����ֱ���ô��ţ������ô���
enum
{
	kLocationLeft = 0,	//0 ����
	kLocationBottom,	//1 ����
	kLocationRight,		//2 ����
	kLocationTop,		//3 ����
	kLocationCenterVL,	//4 ������ֱƫ��
	kLocationCenterVR,	//5 ������ֱƫ��
	kLocationCenterHB,	//6 ����ˮƽƫ��
	kLocationCenterHT,	//7 ����ˮƽƫ��
  
	kLocationTDX,		//8 ��άX��
	kLocationTDY,		//9 ��άY��
	kLocationTDZ,		//10��άZ��
	
	kLocationCount
};

// ͼ��λ��
enum
{
	kLegendArbitrary,									//0 �ڲ�����λ�ã����϶�
  
	kLegendInnerLeft,									//1 ����
	kLegendInnerLeftTop,								//2 ������
	kLegendInnerTopLeft = kLegendInnerLeftTop,			//2 ������
	kLegendInnerLeftBottom,								//3 ������
	kLegendInnerBottomLeft = kLegendInnerLeftBottom,	//3 ������
	kLegendInnerRight,									//4 ����
	kLegendInnerRightTop,								//5 ������
	kLegendInnerTopRight = kLegendInnerRightTop,		//5 ������
	kLegendInnerRightBottom,							//6 ������
	kLegendInnerBottomRight = kLegendInnerRightBottom,	//6 ������
	kLegendInnerTop,									//7 ����
	kLegendInnerBottom,									//8 ����
	
	kLegendLeft,										//9 ��
	kLegendLeftTop,										//10����
	kLegendLeftBottom,									//11����
	kLegendRight,										//12��
	kLegendRightTop,									//13����
	kLegendRightBottom,									//14����
	kLegendTop,											//15��
	kLegendTopLeft,										//16����
	kLegendTopRight,									//17����
	kLegendBottom,										//18��
	kLegendBottomLeft,									//19����
	kLegendBottomRight,									//20����
  
	kLegendPostionCount									//��21��ѡ��
};

// ͼ����������ʱ����������
// ���ȡ��ע�ͣ�����ֱ���ô��ţ������ô���
enum
{
	kZoomCenterLT,		//0 ����
	kZoomCenterLB,		//1 ����
	kZoomCenterRT,		//2 ����
	kZoomCenterRB,		//3 ����
	kZoomCenterCT,		//4 ����
	kZoomCenterARB,		//5 ����㣬��Ҫ�������λ��
  
	kZoomCenterCount
};

// ��ͼ��ʼλ��
enum
{
	kStemBaseBottom		=	0,		//��
	kStemBaseTop		=	1,		//��
	kStemBaseLeft		=	2,		//��
	kStemBaseRight		=	3,		//��
  
	kStemBaseCount		=	4
};
 // how to show the mouse tool tip
 enum
{
	kDesCoords,
	kDesXY,
	kDesElements,
	kDesXAndYAll,
		
	kDesCount
};

// �Ի����Ҽ��˵�ʹ�õ�����
enum
{
	kLangEnglish,		//0 Ӣ��
	kLangChinese,		//1 ����
	kLangCount
};
*/

///////////////////////////////////////////////////////////////////////////////////////////
// Declaration of CChart

class	CChart_API	CChart
{
public:	
	// CChart�ڲ����ݽṹ
	struct CChartPara;
	// ����ڲ����ݵ�ָ��
	CChartPara *GetPara();
protected:
	// �ڲ�����
	CChartPara	*m_pPara;
	// �ͷ��ڲ���Դ
	void		Release();	

public:
	CChart();
	virtual	~CChart();

public:
	// GDI+�Ƿ��ʼ��
	static	bool	IsGdiPlusInitialized();
	// ��ʼ��GDI+
	static	void	InitGdiPlus();
	// ��СGDI+��ʼ�������������С��0�������ͷ�
	static	void	FreeGdiPlus();
	// �ͷ�GDI+
	//static	void	FinalizeGdiPlus();
	
	// ���öԻ����Ҽ��˵�ʹ�õ�����
	static	void	SetLangurage(int nLang);

	// �����Ƿ���ʾʾ����ģʽ�Ĳ˵��ͶԻ���
	static	void	SetOscilloMenus(bool bOscillo);

	// �������ݴ����˵��Ƿ���ʾ
	static	void	SetDataProcessing(bool process);

	// �����Ƿ����ù�����ʾ
	static	bool	IsEnableToolTip();
	// �����Ƿ����ù�����ʾ
	static	void	SetEnableToolTip(bool enable);
	// �����ͼ���Ƿ���Ч
	static	bool	CheckPlotType(int nType);	

	// ����Ĭ����ɫ���ñ�
	static	void	SetColorTable(int nTableIndex);

public:
	// ����ͼ�񵽼�����
	void		CopyImageToClipBoard(HWND hWnd);
	// ����ͼ���ļ�
	void		SaveImageToFile(HWND hWnd, TCHAR *strFileName, TCHAR *strFileExt);
	// ��ӡͼ��
	void		PrintImage(HWND hWnd);

public:

	// �����ͼ����Ƿ��ڷ�Χ��
	bool		CheckSubPlotIndex(int nPlotIndex);

public:
	// ��ù�����ʾ�ַ���
	TCHAR		*GetDescriptionByMousePos(HDC hDC, POINT point);
	// ���ù�����ʾ�ַ�������
	void		SetToolTipType(int type);

public:
	// �����Ƿ���Ӧ��������Ϣ
	bool		IsReact();
	// �����Ƿ���Ӧ��������Ϣ
	void		SetReact(bool react);
	// �����Ƿ���Ӧ�������������Ϣ
	void		SetReactLButtonDown(bool react);
	// �����Ƿ���Ӧ�������̧����Ϣ
	void		SetReactLButtonUp(bool react);
	// �����Ƿ���Ӧ���˫����Ϣ
	void		SetReactLButtonDblClk(bool react);
	// �����Ƿ���Ӧ����ƶ���Ϣ
	void		SetReactMouseMove(bool react);
	// �����Ƿ���Ӧ�Ҽ��˵���Ϣ
	void		SetReactContextMenu(bool react);
	// �����Ƿ���Ӧ������Ϣ
	void		SetReactKeyDown(bool react);
	// ������Ϣ��Ӧʱ�Ƿ��Զ�ˢ�£���������ĩβ��R�ļ���������Interactive
	// ��Ҫ����ʵʱ������ʾ,��ʱ����Ϊfalse
	bool		IsAutoRedraw();
	// ������Ϣ��Ӧʱ�Ƿ��Զ�ˢ�£���������ĩβ��R�ļ���������Interactive
	// ��Ҫ����ʵʱ������ʾ,��ʱ����Ϊfalse
	void		SetAutoRedraw(bool redraw);
	// �����Ƿ����ƻ�ͼ��
	bool		IsRectConfined();
	// �����Ƿ����ƻ�ͼ��
	void		SetRectConfined(bool confine);
	// ����˫�����Ƿ�ʹ��
	bool		IsBuffered();
	// ����˫�����Ƿ�ʹ��
	void		SetBuffered(bool buffered);
	// ���û�ͼ��ƫ��
	void		SetOffsetClient(int offsetX, int offsetY);

public:
	// �Դ��ھ����ͼ
	virtual	void		OnDraw(HWND hWnd);
	// ���豸�����Ļ�ͼ
	virtual	void		OnDraw(HDC hDC);
	// �ڴ�����ѡ�������ͼ
	virtual	void		OnDraw(HDC hDC, RECT destRect);
	// ���豸�����Ķ�Ӧ�Ĵ����ϣ�ѡ�������ͼ
	virtual	void		OnDraw(HWND hWnd, RECT destRect);

public:
	// ��������������Ϣ�������ػ棬���û����ݷ���ֵ�ػ�
	virtual	bool		OnLButtonDown( HWND hWnd, POINT point, UINT ctrlKey );
	virtual	bool		OnLButtonUp( HWND hWnd, POINT point, UINT ctrlKey );
	virtual	bool		OnLButtonDblClk( HWND hWnd, POINT point, UINT ctrlKey );
	virtual	bool		OnMouseMove( HWND hWnd, POINT point, UINT ctrlKey );
	virtual	bool		OnMouseLeave( HWND hWnd, POINT point, UINT ctrlKey );
	virtual	bool		OnMouseWheel( HWND hWnd, POINT point, UINT ctrlKey );
	virtual	bool		OnContextMenu( HMENU hMenu, HWND hWnd, POINT point );
	virtual	bool		OnKeyDown( HWND hWnd, UINT key );

	// ��������������Ϣ����AutoRedrawΪ��������ػ�
	void				OnLButtonDownR( HWND hWnd, POINT point, UINT ctrlKey );
	void				OnLButtonUpR( HWND hWnd, POINT point, UINT ctrlKey );
	void				OnLButtonDblClkR( HWND hWnd, POINT point, UINT ctrlKey );
	void				OnMouseMoveR( HWND hWnd, POINT point, UINT ctrlKey );
	void				OnMouseLeaveR( HWND hWnd, POINT point, UINT ctrlKey );
	void				OnMouseWheelR( HWND hWnd, POINT point, UINT ctrlKey );
	void				OnContextMenuR( HMENU hMenu, HWND hWnd, POINT point );
	void				OnKeyDownR( HWND hWnd, UINT key );

	// ��Ӧ��Ϣ�������ػ棬���û����ݷ���ֵ�ػ�
	virtual	bool		OnEvent(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
	// ��Ӧ��Ϣ����AutoRedrawΪ��������ػ�
	void				Interactive(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

	// �����Ҽ��˵������˵�����
	void		SetPluginMenu(void (*fcnAddPluginMenu)( void *plot, HMENU addMenu ), int (*fcnInterpretPluginMenu)( void *plot, HWND hWnd, int result ), void *pPara);
	// ���ػ���ʾȱʡ�Ҽ��˵�����ϲ���˵����ܿ�ʵ���Ҽ��˵�ȫ���Զ���
	void		SetDefMenus(bool bShow);
	// �û��Զ����ͼ����
	void		SetUserDrawingFunc(void	(*fcnUserDrawing)( HDC hDC, RECT clientRect, RECT plotRect, void *pPara ), void *pPara);
	// �û��Զ����ͼ����
	void		SetUserDrawingFunc(void	(*fcnUserDrawing)( HDC hDC, RECT clientRect, RECT plotRect, void *pPara ), void *pPara, int nPlotIndex);
	// �û��Զ�����²���
	void		SetUserUpdateFunc(void	(*fcnUserUpdate)( void *pPara ), void *pPara);
	// �û��Զ�����²���
	void		SetUserUpdateFunc(void	(*fcnUserUpdate)( void *pPara ), void *pPara, int nPlotIndex);

	// �û��Զ�������ƶ�ǰ����������true����Ĭ������
	void		SetPreMouseMoveFunc(bool	(*fcnPreMouseMove)( void *plot, HDC hDC, POINT point, UINT ctrlKey, void *para ), void *pPara);
	// �û��Զ�����굥��ǰ����������true����Ĭ������
	void		SetPreLButtonDownFunc(bool	(*fcnPreLButtonDown)( void *plot, HDC hDC, POINT point, UINT ctrlKey, void *para ), void *pPara);
	// �û��Զ������̧��ǰ����������true����Ĭ������
	void		SetPreLButtonUpFunc(bool	(*fcnPreLButtonUp)( void *plot, HDC hDC, POINT point, UINT ctrlKey, void *para ), void *pPara);
	// �û��Զ������˫��ǰ����������true����Ĭ������
	void		SetPreLButtonDblClkFunc(bool	(*fcnPreLButtonDblClk)( void *plot, HDC hDC, POINT point, UINT ctrlKey, void *para ), void *pPara);
	// �û��Զ��尴��ǰ����������true����Ĭ������
	void		SetPreKeyDownFunc(bool	(*fcnPreKeyDown)( void *plot, HDC hDC, UINT key, void *para ), void *pPara);
	
	// �û��Զ�������ƶ������������ֵû�й�ϵ
	void		SetPostMouseMoveFunc(bool	(*fcnPostMouseMove)( void *plot, HDC hDC, POINT point, UINT ctrlKey, void *para ), void *pPara);
	// �û��Զ�����굥�������������ֵû�й�ϵ
	void		SetPostLButtonDownFunc(bool	(*fcnPostLButtonDown)( void *plot, HDC hDC, POINT point, UINT ctrlKey, void *para ), void *pPara);
	// �û��Զ������̧������������ֵû�й�ϵ
	void		SetPostLButtonUpFunc(bool	(*fcnPostLButtonUp)( void *plot, HDC hDC, POINT point, UINT ctrlKey, void *para ), void *pPara);
	// �û��Զ������˫�������������ֵû�й�ϵ
	void		SetPostLButtonDblClkFunc(bool	(*fcnPostLButtonDblClk)( void *plot, HDC hDC, POINT point, UINT ctrlKey, void *para ), void *pPara);
	// �û��Զ��尴�������������ֵû�й�ϵ
	void		SetPostKeyDownFunc(bool	(*fcnPreKeyDown)( void *plot, HDC hDC, UINT key, void *para ), void *pPara);

public:
	// �����ͼ��ţ���������kTypeSplit
	int			GetPlotIndexByMousePos(POINT point);

public:
	// ������û�����ݵ�ʱ���Ƿ������ἰ����
	void		SetDrawBasicAnyway(bool draw);
	// ������û�����ݵ�ʱ����ͼ�Ƿ������ἰ����
	void		SetDrawBasicAnyway(bool draw, int nPlotIndex);
	// ����������ʱĬ����Ļ
	void		SetDefScreen( void (*fcnDefScreen)( void *plot, HDC hDC, RECT plotRect, void *pPara ), void *pPara );
	// ����������ʱ��ͼĬ����Ļ
	void		SetDefScreen( void (*fcnDefScreen)( void *plot, HDC hDC, RECT plotRect, void *pPara ), void *pPara, int nPlotIndex );

public:
	// ��ȡ�ڲ�Plotָ��
	void		*GetPlot();
	// ��ȡͼ�����ͱ���
	int			GetType();
	// ����ͼ�����ͱ��룬ע�⽫�����������
	bool		SetType(int nType);
	// ����ͼ�����ͱ��룬�����û�ͼ���ڣ���Ҫ������ά��ͼ����ά��ͼ������hWnd
	bool		SetType(int nType, HWND hWnd);
	
	// ������������
	bool		SetConfineRect(RECT ConfineRect);
	// ��ȡ��������
	RECT		GetConfinedRect();

	// �����ڲ��Ƿ�������
	bool		IsEmpty();

	// �����������
	void		ResetApperance();
	// ������ģʽ���
	int			GetApperanceMode();
	// �������ģʽ
	void		SetApperanceMode(int mode);

public:	
	// �������ߣ�ǰ������������������
	// ���ĸ�������Ҫ���ڷ�����ͼ���ֲ���ͼ������X����ͼ����ʾ��ͼ��š�����������ͬ
	// ����ֵ�����ߵ�ID�ţ�ע�ⲻ����ţ�����-1��ʾ��������ʧ�ܡ�����������ͬ
	int			AddCurve(double *pX, double *pY, int nLen, int nPlotIndex=0);
	// �������ߣ�����X����Ϊʱ�䣬���ַ�����ʽ����
	// �ַ���ӦΪ"20130528234428"����������ʱ���룬��14λ���֣��������
	// ʱ��ֻ�ܾ�ȷ���룬��Ϊ�ڲ�ʹ��time_t
	int			AddCurve(TCHAR **pStrTime, double *pY, int nLen, int nPlotIndex=0);
	// ���ӿ�����
	int			AddCurve(int nPlotIndex=0);
	// �ٲ�ͼר��
	int			AddCurve(double *pX, double *pY, double z, int nLen, int nPlotIndex=0);
	// ������������
	int			UpdateCurve(int nDataIndex, double *pX, double *pY, int nLen, int nPlotIndex=0);
	// ����ID�Ÿ�����������
	int			UpdateCurveByID(int nDataID, double *pX, double *pY, int nLen, int nPlotIndex=0);
	// �����ߵ�DataID���Index
	int			GetIndex(int dataID, int nPlotIndex=0);
	// �����ߵ�Index���DataID
	int			GetDataID(int nIndex, int nPlotIndex=0);
	// �������ߵ�dataID��������;���������콣����������Ҫ������
	void		SetDataID(int newID, int nIndex, int nPlotIndex=0);
	// ���ӱ�ͼ��һ��ֵ
	int			AddPie(double value);
	// ���ӱ�ͼ��һ��ֵ��ͬʱ���øÿ���ı���
	int			AddPie(double value, TCHAR* title);
	// ������ͼ��һ������
	int			AddStems(double *pData, int nLen);
	// ������ͼ��һ�����У�ͬʱ���ø����еı���
	int			AddStems(double *pData, int nLen, TCHAR* title);
	// ����2ά���ߵ�һ�����ݵ�
	// nDataIndex��ʾ���ߵ����,
	// nDataIndex������ڵ�ǰ���ߵ�������������һ�����ߣ����С��0���ߴ��ڵ�ǰ����������������ʧ��
	int			AddPoint2D(double x, double y, int nDataIndex=0, int nPlotIndex=0);
	// �������ݵ㣬����X����Ϊʱ�䣬���ַ�����ʽ����
	//�ַ���ӦΪ"20130528234428"����������ʱ���룬��14λ���֣��������
	// ʱ��ֻ�ܾ�ȷ���룬��Ϊ�ڲ�ʹ��time_t
	int			AddPoint2D(TCHAR *strTime, double y, int nDataIndex=0, int nPlotIndex=0);
	// ����2ά���ߵ�һ�����ݵ�
	// nPos��ʾ��Ҫ�����λ��,
	int			InsertPoint2D(double x, double y, int nPos, int nDataIndex=0, int nPlotIndex=0);
	
	// ������ά���ߵ�һ�����ݵ�
	int			AddPoint3D(double x, double y, double z, int nDataIndex=0);
	// ����һ����ά����
	int			AddCurve(double *pX, double *pY, double *pZ, int nLen);	
	
	// ���ó����������ڵȸ���ͼ����ͼ����ά����ͼ
	void		SetFieldFcn(double (*_pFieldFcn) (double, double));
	// ��ó�����ָ�룬���ڵȸ���ͼ����ͼ����ά����ͼ
	double		(*GetFieldFcn( ))( double, double );
	// ���ӵȸ������ݵ�
	void		AddContourPoint(double x, double y, double h);
	// ����ȸ������ݵ�
	void		ClrContourPoints();

	// ���������������
	void		ClrAllData();
	// �������ͼȫ����������
	void		ClrPlotData(int nPlotIndex=0);
	// ���������������
	void		ClrSingleData(int nDataIndex, int nPlotIndex=0);
	// ��յ�����������
	void		EmptySingleData(int nDataIndex, int nPlotIndex=0);
	// Ϊ����Ԥ���ڴ�ռ䡣��ҪĿ���Ǽӿ��ٶ�
	// CChart�ڲ����ݲ���vector��������ݵ���࣬�������ݵ�����ӣ�CChart�����ϵ�Ϊvector���·����ڴ棬Ӱ���ٶ�
	// �������ʵ�����Ԥ�����ڴ棬���һ���Գɹ������ڴ�
	void		SetReservedDataLength(int nLen, int nPlotIndex=0);
	// ������ߵ����ݵ���
	int			GetDataPointsCount(int nDataIndex, int nPlotIndex=0);
	// ��ȡ���ݵ�x
	double		GetDataPointX(int nPointIndex, int nDataIndex, int nPlotIndex=0);
	// ��ȡ���ݵ�y
	double		GetDataPointY(int nPointIndex, int nDataIndex, int nPlotIndex=0);
	//��ȡ���ݵ�
	bool		GetDataPoint(double data[2], int nPointIndex, int nDataIndex, int nPlotIndex=0);
	// ȥ�������ϵ������ݵ�
	int			RemoveSinglePoint(int nPointIndex, int nDataIndex, int nPlotIndex=0);
	// ȥ�������ϵ�һ�����ݵ�
	int			RemoveFirstPoint(int nDataIndex, int nPlotIndex=0);
	// ȥ�������ϵ�һ�����ݵ㣬ͬʱ�ڲ��������ƣ��ɱ�֤ռ���ڴ��λ�ò��䣬��Ч�ʱ�RemoveFirstPoint�͡�
	int			RemoveFirstPointAndShift(int nDataIndex, int nPlotIndex=0);
	// ȥ�����������һ�����ݵ�
	int			RemoveLastPoint(int nDataIndex, int nPlotIndex=0);
	// ����X����
	int			SlipXValue(int nStep, int nDataIndex, int nPlotIndex=0);
	// ����Y����
	int			SlipYValue(int nStep, int nDataIndex, int nPlotIndex=0);

public:
	// ����һ��ͼ�㣬���ڷֲ���ͼ������X����ͼ
	int			AddLayer();
	// ������ͼ���������ڷ�����ͼ������mode��ʾ����ģʽ��nRows��ʾ������nCols��ʾ����
	// ����ģʽ�μӱ��ļ�ͷ����ע�͵���enum�����Խ��ע�ͣ�����ֱ��ʹ����Щenum�ˡ�
	void		ResizePlots(int mode, int nRows, int nCols);
	// ����ͼ�����������ڷֲ���ͼ������X����ͼ
	void		ResizePlots(int nLayers);
	// ���÷�����ͼ��ͼ��
	static	bool		ResizeSplit(CChart *pChart, int nSubPlots);
	
public:
	// ���ͼ�����
	TCHAR*		GetTitle();
	// �����ͼ���⣬�����ڷ�����ͼ���ֲ���ͼ������X����ͼ
	TCHAR*		GetTitle(int nPlotIndex);
	// ����ͼ�����
	void		SetTitle(const TCHAR* title);
	// ������ͼ����
	void		SetTitle(const TCHAR* title, int nPlotIndex);
	// ���ͼ�����λ��
	int			GetTitlePosition();
	// �����ͼ����λ��
	int			GetTitlePosition(int nPlotIndex);
	// ����ͼ�����λ�á���������0���У���������
	void		SetTitlePosition(int position);
	// ������ͼ����λ�á���������0���У���������
	void		SetTitlePosition(int position, int nPlotIndex);
	// ͼ������Ƿ���ʾ
	bool		IsTitleShow();
	// ��ͼ�����Ƿ���ʾ
	bool		IsTitleShow(int nPlotIndex);
	// ������ʾͼ�����
	void		SetTitleShow(bool show);
	// ������ʾ��ͼ����
	void		SetTitleShow(bool show, int nPlotIndex);
	// ���ͼ�������ɫ
	COLORREF	GetTitleColor();
	// �����ͼ������ɫ
	COLORREF	GetTitleColor(int nPlotIndex);
	// ����ͼ�������ɫ
	void		SetTitleColor(COLORREF color);
	// ������ͼ������ɫ
	void		SetTitleColor(COLORREF color, int nPlotIndex);
	// ���ͼ�񸱱���
	TCHAR*		GetSubTitle();
	// �����ͼ�����⣬�����ڷ�����ͼ���ֲ���ͼ������X����ͼ
	TCHAR*		GetSubTitle(int nPlotIndex);
	// ����ͼ�񸱱���
	void		SetSubTitle(const TCHAR* title);
	// ������ͼ������
	void		SetSubTitle(const TCHAR* title, int nPlotIndex);
	// ���ͼ�񸱱���λ��
	int			GetSubTitlePosition();
	// �����ͼ������λ��
	int			GetSubTitlePosition(int nPlotIndex);
	// ����ͼ�񸱱���λ�á���������0���У���������
	void		SetSubTitlePosition(int position);
	// ������ͼ������λ�á���������0���У���������
	void		SetSubTitlePosition(int position, int nPlotIndex);
	// ͼ�񸱱����Ƿ���ʾ
	bool		IsSubTitleShow();
	// ��ͼ�������Ƿ���ʾ
	bool		IsSubTitleShow(int nPlotIndex);
	// ������ʾͼ�񸱱���
	void		SetSubTitleShow(bool show);
	// ������ʾ��ͼ������
	void		SetSubTitleShow(bool show, int nPlotIndex);
	// ���ͼ�񸱱�����ɫ
	COLORREF	GetSubTitleColor();
	// �����ͼ��������ɫ
	COLORREF	GetSubTitleColor(int nPlotIndex);
	// ����ͼ�񸱱�����ɫ
	void		SetSubTitleColor(COLORREF color);
	// ������ͼ��������ɫ
	void		SetSubTitleColor(COLORREF color, int nPlotIndex);
	// �ֲ���ͼ��ʾ������
	void		SetShowParentTitle(bool set);
	// ��ñ�������
	LOGFONT		GetTitleFont();
	// �����ͼ��������
	LOGFONT		GetTitleFont(int nPlotIndex);
	// ���ñ�������
	void		SetTitleFont(LOGFONT logFont);
	// ������ͼ��������
	void		SetTitleFont(LOGFONT logFont, int nPlotIndex);
	// ��ø���������
	LOGFONT		GetSubTitleFont();
	// �����ͼ����������
	LOGFONT		GetSubTitleFont(int nPlotIndex);
	// ���ø���������
	void		SetSubTitleFont(LOGFONT logFont);
	// ������ͼ����������
	void		SetSubTitleFont(LOGFONT logFont, int nPlotIndex);
	// ���ø����߿���ɫ
	void		SetLightColor(COLORREF color);
	// ������ͼ�����߿���ɫ
	void		SetLightColor(COLORREF color, int nPlotIndex);
	// ���ø����߿��߿�
	void		SetLightLineSize(int nSize);
	// ������ͼ�����߿��߿�
	void		SetLightLineSize(int nSize, int nPlotIndex);
	// ���ø����߿�����
	void		SetLightLineStyle(int nStyle);
	// ������ͼ�����߿�����
	void		SetLightLineStyle(int nStyle, int nPlotIndex);

	// �����ͼ����
	int			GetSubPlotsCount();
	// ����������������ڶ����ͼΪ��ͼ��������
	int			GetDataSetCount(int nPlotIndex=0);
	// ���õ��������Ƿ���Ӧ��������Ϣ
	void		SetReact(bool react, int nDataIndex, int nPlotIndex=0);
	// ����������ߵı���,nDataIndex��ʾ���ߵ����
	const TCHAR*		GetDataTitle(int nDataIndex, int nPlotIndex=0);
	// �������ݱ���
	void		SetDataTitle(const TCHAR* title, int nDataIndex, int nPlotIndex=0);
	// ���������ɫ
	COLORREF	GetDataColor(int nDataIndex, int nPlotIndex=0);
	// ����������ɫ
	void		SetDataColor(COLORREF color, int nDataIndex, int nPlotIndex=0);
	// �Ƿ�˫ɫ����ģʽ
	bool		IsBiColorMode(int nDataIndex, int nPlotIndex=0);
	// ����˫ɫ����ģʽ
	void		SetBiColorMode(bool bBi, int nDataIndex, int nPlotIndex=0);
	// ���˫ɫ����ģʽ�µڶ�������ɫ
	COLORREF	GetDataColor2(int nDataIndex, int nPlotIndex=0);
	// ����˫ɫ����ģʽ�µڶ�������ɫ
	void		SetDataColor2(COLORREF color, int nDataIndex, int nPlotIndex=0);
	// �Ƿ��ɫ����ģʽ��������˫ɫ����
	bool		IsMultiColorMode(int nDataIndex, int nPlotIndex=0);
	// ���ö�ɫ����ģʽ
	void		SetMultiColorMode(bool bMul, int nDataIndex, int nPlotIndex=0);
	// ��ö�ɫ����ģʽ������ڵ�ķ�����0�����ݵ����0.0-1.0��1��X����ֵ��2��Y����ֵ
	int			GetMultiColorInputType(int nDataIndex, int nPlotIndex=0);
	// ���ö�ɫ����ģʽ������ڵ�ķ���
	void		SetMultiColorInputType(int nType, int nDataIndex, int nPlotIndex=0);
	// ���ö�ɫ����ģʽ�Ľڵ㣬����ratio�ĵķ�Χ�����뷽������
	void		AddSegmentColor(double ratio, COLORREF color, int nDataIndex, int nPlotIndex=0);
	// �Ƿ������ݵ㶨ɫ
	bool		IsColorPtByPt(int nDataIndex, int nPlotIndex=0);
	// �Ƿ������ݵ㶨ɫ
	void		SetColorPtByPt(bool bBy, int nDataIndex, int nPlotIndex=0);
	// �������ݵ���ɫ��ע�⣬���ӵ���ɫ���ݵ��������ݵ������ȫһ������Ȼû������
	void		AddDataPointColor(COLORREF color, int nDataIndex, int nPlotIndex=0);
	// �������ݵ���ɫ
	void		SetDataPointColor(COLORREF color, int nPointIndex, int nDataIndex, int nPlotIndex=0);
	// ɾ��һ�����ݵ���ɫ
	void		EraseDataPointColor(int nPointIndex, int nDataIndex, int nPlotIndex=0);
	// ����һ�����ݵ���ɫ
	void		InsertDataPointColor(COLORREF color, int nPointIndex, int nDataIndex, int nPlotIndex=0);
	// �����������
	int			GetDataLineStyle(int nDataIndex, int nPlotIndex=0);
	// ������������
	void		SetDataLineStyle(int nStyle, int nDataIndex, int nPlotIndex=0);
	// ��������߿�
	int			GetDataLineSize(int nDataIndex, int nPlotIndex=0);
	// ���������߿�
	void		SetDataLineSize(int nSize, int nDataIndex, int nPlotIndex=0);
	// �������ߵĻ��Ʒ�ʽ
	void		SetPlotType(int nType, int nDataIndex, int nPlotIndex=0);
	// �����������ݵ����Ƿ���ʾ
	void		SetMarkerShow(bool bShow, int nDataIndex, int nPlotIndex=0);
	// �����������ݵ����Ƿ����
	void		SetMarkerFill(bool bFill, int nDataIndex, int nPlotIndex=0);
	// �����������ݵ��ǵ���״
	void		SetMarkerType(int nType, int nDataIndex, int nPlotIndex=0);
	// �����������ݵ�Ĵ�С
	void		SetMarkerSize(int nSize, int nDataIndex, int nPlotIndex=0);
	// �����������ݵ���Ƶ�ʣ�0��ʾ����ǣ���n��ʾÿ��n������һ�Σ���n��ʾһ�����n����
	void		SetMarkerFreq(int nFreq, int nDataIndex, int nPlotIndex=0);
	// �����������ݵ��ǵ���ʼ�㣬���ڲ���ȫ�����ʱ
	void		SetMarkerStartIndex(int nStartIndex, int nDataIndex, int nPlotIndex=0);
	// �����������ݵ��һ������
	void		SetMarkFirstPoint(bool bMark, int nPlotIndex=0);
	// �����������ݵ����һ������
	void		SetMarkLastPoint(bool bMark, int nPlotIndex=0);
	// ��������ɢ����ʾģʽ
	void		SetScatter(int nDataIndex, int nPlotIndex=0);
	// �������������ɫ
	void		SetDataFillColor(bool bFill, COLORREF color, int nOpaque, int nFillMode, int nDataIndex, int nPlotIndex=0);
	// �����������ݰ���ʾ
	void		SetDataBarMode(bool bShowBar, int nBarBase, int nBarSize, int nDataIndex, int nPlotIndex=0);
	// ����ѡ��͸���ģʽ
	void		SetSLMode(int nMode, int nDataIndex, int nPlotIndex=0);
	// ��ȡ���ݵı�־��������;
	bool		GetDataFlag(int nDataIndex, int nPlotIndex=0);
	// �������ݵı�־��������;
	void		SetDataFlag(bool bFlag, int nDataIndex, int nPlotIndex=0);
	// ��ȡ���ݿɼ���־
	bool		IsDataVisible(int nDataIndex, int nPlotIndex=0);
	//���������Ƿ�ɼ�
	void		SetDataVisible(bool bVis, int nDataIndex, int nPlotIndex=0);
	// ����ѹ����ʾģʽ����Ҫ���ڴ�������ʱ����ʾ
	// ѹ����ʾģʽ�У�����������ݵ�����Ļ����ʾʱ��X��ͬ����ѹ����һ�����ݵ㣬ֵȡƽ����
	void		SetDataCompactDraw(bool bCompact, int nDataIndex, int nPlotIndex=0);
	// �������ݰ�ֱ��ͼģʽ��ʾ
	void		SetDataHistro(bool bHistro, int nDataIndex, int nPlotIndex=0);

	// �����������ݵ���ֵ�Ƿ���ʾ
	void		SetDataValueShow(bool bShow, int nDataIndex, int nPlotIndex=0);
	// �����������ݵ���ֵ��ʾ��ʽ��1ΪX��2ΪY��3ΪX:Y������Ϊ����ʾ
	void		SetDataValueType(int nType, int nDataIndex, int nPlotIndex=0);
	// �����������ݵ���ֵ�������С
	void		SetDataValueFontSize(int nSize, int nDataIndex, int nPlotIndex=0);
	// �����������ݵ���ֵ��ʾƵ�ʣ�0��ʾ����ʾ����n��ʾÿ��n������ʾһ�Σ���n��ʾһ����ʾn����
	void		SetDataValueFreq(int nFreq, int nDataIndex, int nPlotIndex=0);
	// �����������ݵ���ֵ��ʾ����ʼ�㣬���ڲ���ȫ����ʾʱ
	void		SetDataValueStartIndex(int nStartIndex, int nDataIndex, int nPlotIndex=0);

	// ���ʾ����ģʽ
	bool		IsOscilloMode(int nPlotIndex=0);
	// ����ʾ����ģʽ
	void		SetOscilloMode(bool bOscillo, int nPlotIndex=0);
	// ����Ƿ�����϶�����
	bool		IsEnableDataDrag();
	// �����Ƿ�����϶�����
	void		SetEnableDataDrag(bool enable);
	// ����Ƿ�����϶��������
	bool		IsEnableZoneDrag();
	// �����Ƿ�����϶��������
	void		SetEnableZoneDrag(bool enable);
	// �ƶ�����
	bool		MoveData(double offsetx, double offsety, int nDataIndex, int nPlotIndex=0);
	// ������ߵ�ˮƽƫ��
	double		GetDataOffsetX(int nDataIndex, int nPlotIndex=0);
	// �������ߵ�ˮƽƫ��
	void		SetDataOffsetX(double offset, int nDataIndex, int nPlotIndex=0);
	// ������ߵ���ֱƫ��
	double		GetDataOffsetY(int nDataIndex, int nPlotIndex=0);
	// �������ߵ���ֱƫ��
	void		SetDataOffsetY(double offset, int nDataIndex, int nPlotIndex=0);
	// ����X����λ����������ģ��ʾ����
	void		SetXUnit(TCHAR *unit, int nDataIndex);
	// ����Y����λ����������ģ��ʾ����
	void		SetYUnit(TCHAR *unit, int nDataIndex);
	// ��ͼ���н�����ʾY����ı����ߣ���������ģ��ʾ����
	void		SetShowYSCaleOnly(bool only);
	// �����󶨵����
	bool		IsHoldCursorToMouse(int nPlotIndex=0);
	// ���ù��󶨵����
	void		SetHoldCursorToMouse(bool hold, int nPlotIndex=0);
	// �����󶨵�����
	bool		IsHoldCursorToCurve(int nPlotIndex=0);
	// ���ù��󶨵�����
	void		SetHoldCursorToCurve(bool hold, int nPlotIndex=0);
	// ��ù��󶨵��������
	int			GetIndexToHoldCursor(int nPlotIndex=0);
	// ���ù��󶨵��������
	void		SetIndexToHoldCursor(int nCurveIndex, int nPlotIndex=0);
	// �������ͷ����
	bool		IsShowZeroArrow(int nPlotIndex=0);
	// ��������ͷ����
	void		SetShowZeroArrow(bool show, int nPlotIndex=0);
	// ��ⴥ����ͷ����
	bool		IsShowTrigArrow(int nPlotIndex=0);
	// ���ô�����ͷ����
	void		SetShowTrigArrow(bool show, int nPlotIndex=0);
	// ��ʾˮƽ����ߣ���������ģ��ʾ����
	void		SetShowHCursor(bool show, int nPlotIndex=0);
	// ��ʾ��ֱ����ߣ���������ģ��ʾ����
	void		SetShowVCursor(bool show, int nPlotIndex=0);
	// ��ù������ɫ����������ģ��ʾ����
	COLORREF	GetCursorColor(int nPlotIndex=0);
	// ���ù������ɫ����������ģ��ʾ����
	void		SetCursorColor(COLORREF color, int nPlotIndex=0);
	// ��ù�����߿�����������ģ��ʾ����
	int			GetCursorSize(int nPlotIndex=0);
	// ���ù�����߿�����������ģ��ʾ����
	void		SetCursorSize(int size, int nPlotIndex=0);
	// ��ù�������ͣ���������ģ��ʾ����
	int			GetCursorStyle(int nPlotIndex=0);
	// ���ù�������ͣ���������ģ��ʾ����
	void		SetCursorStyle(int style, int nPlotIndex=0);
	// ���ˮƽ���ֵ����������ģ��ʾ����
	double		GetCursorX(int nPlotIndex=0);
	// ����ˮƽ���ֵ����������ģ��ʾ����
	void		SetCursorX( double cursor, int nPlotIndex=0);
	// �����ֱ���ֵ����������ģ��ʾ����
	double		GetCursorY(int nPlotIndex=0);
	// ������ֱ���ֵ����������ģ��ʾ����
	void		SetCursorY( double cursor, int nPlotIndex=0);
	// ��ʾˮƽѡ��������������ģ��ʾ����
	void		SetShowHSel(bool show, int nPlotIndex=0);
	// ��ʾ��ֱѡ��������������ģ��ʾ����
	void		SetShowVSel(bool show, int nPlotIndex=0);
	// ���ˮƽѡ���������ޣ���������ģ��ʾ����
	double		GetHSelLower(int nPlotIndex=0);
	// ����ˮƽѡ���������ޣ���������ģ��ʾ����
	void		SetHSelLower(int val, int nPlotIndex=0);
	// ���ˮƽѡ���������ޣ���������ģ��ʾ����
	double		GetHSelUpper(int nPlotIndex=0);
	// ����ˮƽѡ���������ޣ���������ģ��ʾ����
	void		SetHSelUpper(int val, int nPlotIndex=0);
	// �����ֱѡ���������ޣ���������ģ��ʾ����
	double		GetVSelLower(int nPlotIndex=0);
	// ������ֱѡ���������ޣ���������ģ��ʾ����
	void		SetVSelLower(int val, int nPlotIndex=0);
	// �����ֱѡ���������ޣ���������ģ��ʾ����
	double		GetVSelUpper(int nPlotIndex=0);
	// ������ֱѡ���������ޣ���������ģ��ʾ����
	void		SetVSelUpper(int val, int nPlotIndex=0);
	// ���ˮƽѡ�����Ŀ��ȣ���������ģ��ʾ����
	double		GetHSelWidth(int nPlotIndex=0);
	// �����ֱѡ�����Ŀ��ȣ���������ģ��ʾ����
	double		GetVSelWidth(int nPlotIndex=0);
	
	// ���������ᣬ�������λ��location�����ļ�ͷ����ע�͵���enum
	void		AddAxis(int location, int nPlotIndex=0);
	// ������ͼ���������
	void		SetAxisTitle(const TCHAR* title, int location, int nPlotIndex);
	// �������������
	void		SetAxisTitle(const TCHAR* title, int location);
	// ������ͼ�������������
	void		SetAxisTitleFont(LOGFONT logFont, int location, int nPlotIndex);
	// �����������������
	void		SetAxisTitleFont(LOGFONT logFont, int location);
	// ������ͼ�������ǩ����
	void		SetAxisLabelFont(LOGFONT logFont, int location, int nPlotIndex);
	// �����������ǩ����
	void		SetAxisLabelFont(LOGFONT logFont, int location);
	// ������ͼ���������λ��
	void		SetAxisTitlePosition(int position, int location, int nPlotIndex);
	// �������������λ��
	void		SetAxisTitlePosition(int position, int location);
	// ��ͼ�Զ�����������̶���
	void		SetAxisAutoTicks(bool bAuto, int location, int nPlotIndex);
	// �Զ�����������̶���
	void		SetAxisAutoTicks(bool bAuto, int location);
	// ������ͼ������̶���
	void		SetAxisTickCount(int count, int location, int nPlotIndex);
	// ����������̶���
	void		SetAxisTickCount(int count, int location);
	// ��ͼ�Զ����������ḱ�̶���
	void		SetAxisAutoMinorTicks(bool bAuto, int location, int nPlotIndex);
	// �Զ����������ḱ�̶���
	void		SetAxisAutoMinorTicks(bool bAuto, int location);
	// ������ͼ�����ḱ�̶���
	void		SetAxisMinorTickCount(int count, int location, int nPlotIndex);
	// ���������ḱ�̶���
	void		SetAxisMinorTickCount(int count, int location);
	// ������ͼʱ��������
	void		SetAxisToTime(bool bTime, int location, int nPlotIndex);
	// ����ʱ��������
	void		SetAxisToTime(bool bTime, int location);
	// ������ͼʱ���������ʽ,"%Y%m%d%H%M%S",�ο�strftime������CTime��ĸ�ʽ������
	void		SetAxisTimeFormat(TCHAR *format, int location, int nPlotIndex);
	// ����ʱ���������ʽ,"%Y%m%d%H%M%S",�ο�strftime������CTime��ĸ�ʽ������
	void		SetAxisTimeFormat(TCHAR *format, int location);
	// ������ͼ��ʾ��ɫ��
	void		SetAxisColorBar(bool bShow, int location, int nPlotIndex);
	// ������ʾ��ɫ��
	void		SetAxisColorBar(bool bShow, int location);
	// ������ͼ��������ɫ
	void		SetAxisColor(COLORREF color, int location, int nPlotIndex);
	// ������������ɫ
	void		SetAxisColor(COLORREF color, int location);
	// �����������Ƿ���ʾ�̶�ֵ
	void		SetAxisLabelShow(bool bShow, int location);
	// ������ͼ�������Ƿ���ʾ�̶�ֵ
	void		SetAxisLabelShow(bool bShow, int location, int nPlotIndex);
	// ������ͼ����ֵ�ĸ�ʽ,�ο�printf�ĸ�ʽ������
	void		SetAxisLabelFormat(TCHAR *format, int location, int nPlotIndex);
	// ��������ֵ�ĸ�ʽ,�ο�printf�ĸ�ʽ������
	void		SetAxisLabelFormat(TCHAR *format, int location);
	// ������ͼ�����ḡ�����
	void		SetXAxisFloatTicks(bool flt, int nPlotIndex);
	// ���������ḡ�����
	void		SetXAxisFloatTicks(bool flt);
	// ������ͼ�����ḡ�����
	void		SetYAxisFloatTicks(bool flt, int nPlotIndex);
	// ���������ḡ�����
	void		SetYAxisFloatTicks(bool flt);
	// �����������Ƿ���ʾ
	void		SetAxisShow(bool bShow, int location);
	// ������ͼ�������Ƿ���ʾ
	void		SetAxisShow(bool bShow, int location, int nPlotIndex);
	//�������Ƿ���ʾ
	bool		IsAxisShow(int location);
	// ��ͼ�������Ƿ���ʾ
	bool		IsAxisShow(int location, int nPlotIndex);
	// �����������ָ��
	void		SetAxisLogarithm(bool bLog, int location);
	// ������ͼ�������ָ��
	void		SetAxisLogarithm(bool bLog, int location, int nPlotIndex);
	// �������������ģʽ
	void		SetAxesCompact(int nPlotIndex=0);
	// ���������������ǩ��һ�����ݷ�Χ
	void		AddRange4LabelHiden(double low, double high, int location);
	// ������ͼ�����������ǩ��һ�����ݷ�Χ
	void		AddRange4LabelHiden(double low, double high, int location, int nPlotIndex);
	// �����������������ĳ���ı���
	void		SetAtomX(double atom);
	// ������ͼ�������������ĳ���ı���
	void		SetAtomX(double atom, int nPlotIndex);
	// ������������������ĳ���ı���
	void		SetAtomY(double atom);
	// ������ͼ��������������ĳ���ı���
	void		SetAtomY(double atom, int nPlotIndex);
	// ���������귶Χ��ĳ����������£��������ָ����
	void		SetSegsPreferX(int segs);
	// ���������귶Χ��ĳ����������£���ͼ�������ָ����
	void		SetSegsPreferX(int segs, int nPlotIndex);
	// ���������귶Χ��ĳ����������£���������ָ����
	void		SetSegsPreferY(int segs);
	// ���������귶Χ��ĳ����������£���ͼ��������ָ����
	void		SetSegsPreferY(int segs, int nPlotIndex);

	// ����Ƿ�ʹ��ͼ��
	bool		IsUseLegend(int nPlotIndex=0);
	// �����Ƿ�ʹ��ͼ��
	void		SetUseLegend(bool bUse, int nPlotIndex=0);
	// ����ͼ�����������
	void		SetLegendMaxRows(int nRows, int nPlotIndex=0);
	// ����ÿ����ʾ��ͼ����
	void		SetLegendMaxCols(int nCols, int nPlotIndex=0);
	// ����ͼ��λ��
	void		SetLegendPosition(int nPos, int nPlotIndex=0);
	// ����ͼ������
	void		SetLegendFont(LOGFONT logFont, int nPlotIndex=0);
	// ����ͼ������߶�
	void		SetLegendFontHeight(int nHeight, int nPlotIndex=0);

	// ���������᷶Χ
	void		SetPlotRange(double xRange[2], double yRange[2]);
	void		SetPlotRange(double xl, double xu, double yl, double yu);
	void		SetXRange(double low, double high, int nPlotIndex);
	void		SetXRange(double low, double high);
	void		SetYRange(double low, double high, int nPlotIndex);
	void		SetYRange(double low, double high);
	// ���������᷶Χ�Զ�����
	void		SetXAutoRange(bool bAuto);
	void		SetYAutoRange(bool bAuto);
	void		SetXAutoRange(bool bAuto, int nPlotIndex);
	void		SetYAutoRange(bool bAuto, int nPlotIndex);
	// ���������᷶Χ�����ݷ�Χ��ͬ������������չ
	void		SetExactXRange(bool bExact, int nPlotIndex=0);
	void		SetExactYRange(bool bExact, int nPlotIndex=0);
	// ���������᷶Χ���������ݷ�Χ��ͬ
	void		SetOptimalExactXRange(bool bExact, int nPlotIndex=0);
	void		SetOptimalExactYRange(bool bExact, int nPlotIndex=0);
	// ���������᷶Χ������������չһ��
	void		SetOptimalXExtend(bool bExtend, int nPlotIndex=0);
	void		SetOptimalYExtend(bool bExtend, int nPlotIndex=0);

	// ����������ȳ���ֻ��kTypeXY��Ч
	void		SetEqualXYAxis(bool bEqual);
	// ����������ȷ�Χ��ֻ��kTypeXY��Ч
	void		SetEqualXYRange(bool bEqual);
	// ����������ȱ����ߣ�ֻ��kTypeXY��Ч
	void		SetEqualXYScale(bool bEqual);

	// ����ͼ��߿���ʾ
	void		SetEdgeShow(bool bShow);
	// ����ͼ��߿���ʾ
	void		SetEdgeRoundConor(bool bRound);
	// ����ͼ��߿���ɫ
	void		SetEdgeColor(COLORREF color);
	// ����ͼ��߿����
	void		SetEdgeWidth(int width);

	// �����������߿���ʾ
	void		SetBorderShow(bool bShow);
	// �����������߿���ɫ
	void		SetBorderColor(COLORREF color);
	// �����������߿����
	void		SetBorderWidth(int width);

public:
	// ���ñ�ͼ�������ı���
	void		SetPieTitle(const TCHAR* title, int which);
	// ���ñ�ͼ����������ɫ
	void		SetPieColor(COLORREF color, int which);

public:
	// ������ͼ�������ӵı�ǩ
	void		SetStemLabel(const TCHAR* label, int which);
	// ������ͼ�������еı���
	void		SetStemTitle(const TCHAR* title, int nDataIndex);
	// ������ͼ����������ռ����
	void		SetStemRatio(double ratio);
	// ������ͼ�����������Ὺʼ��
	void		SetStemBase(int base);

public:
	// ���õȸ���ͼ�ȸ��ߵ�����
	void		SetContourLineNum(int num);
	// ���õȸ���ͼ�ȸ��߻��Ƶľ��ȣ�nԽ�󣬾���Խ�ߣ�һ��5�Ϳ�����
	void		SetContourPrecision(int n);
	// ������ʾ�ȸ�����ֵ
	void		SetContourValueShow(bool show);

	// ������ͼ����ʼ��ɫ
	void		SetContourMapColor1(COLORREF cr);
	// ������ͼ��������ɫ
	void		SetContourMapColor2(COLORREF cr);

	// ���������ݵ����룬�����ǳ�����
	void		SetContourByPoints();

	// ������ͼ������������Ϊͼ��
	void		SetRightAxisAsLegend(bool as);
	// ������ͼ������������Ϊͼ��
	void		SetTopAxisAsLegend(bool as);

public:
	// ���ñ�����������һ����ͼ�󶨣����ڹ���X����ͼ
	void		SetGridBindLayer(int nPlotIndex);
	// �����ͼ��������
	bool		GetGridLine(bool &MajorH, bool &MajorV, bool &MinorH, bool &MinorV, int nPlotIndex);
	// ���������
	bool		GetGridLine(bool &MajorH, bool &MajorV, bool &MinorH, bool &MinorV);
	// ������ͼ�����ߵĻ���
	void		SetGridLine(bool MajorH, bool MajorV, bool MinorH, bool MinorV, int nPlotIndex);
	// ���������ߵĻ���
	void		SetGridLine(bool MajorH=true, bool MajorV=true, bool MinorH=false, bool MinorV=false);
	// ��������������ɫ
	void		SetMajorGridColor(COLORREF color);
	// ������ͼ����������ɫ
	void		SetMajorGridColor(COLORREF color, int nPlotIndex);
	// ���ø���������ɫ
	void		SetMinorGridColor(COLORREF color);
	// ������ͼ����������ɫ
	void		SetMinorGridColor(COLORREF color, int nPlotIndex);
	// �������������߿�
	void		SetMajorGridLineSize(int nSize);
	// ������ͼ���������߿�
	void		SetMajorGridLineSize(int nSize, int nPlotIndex);
	// ���ø��������߿�
	void		SetMinorGridLineSize(int nSize);
	// ������ͼ���������߿�
	void		SetMinorGridLineSize(int nSize, int nPlotIndex);
	// ����������������
	void		SetMajorGridLineStyle(int nStyle);
	// ������ͼ������������
	void		SetMajorGridLineStyle(int nStyle, int nPlotIndex);
	// ���ø�����������
	void		SetMinorGridLineStyle(int nStyle);
	// ������ͼ������������
	void		SetMinorGridLineStyle(int nStyle, int nPlotIndex);
	// ��ȡ��ͼ������ɫ
	COLORREF	GetBkgndColor(int nPlotIndex);
	// ��ȡ������ɫ
	COLORREF	GetBkgndColor();
	// ������ͼ������ɫ
	void		SetBkgndColor(COLORREF color, int nPlotIndex);
	// ���ñ�����ɫ
	void		SetBkgndColor(COLORREF color);
	// ����ɫ�Ƿ񽥱�
	bool		IsGradientBkgnd();
	// ����ɫ����
	void		SetGradientBkgnd(bool bGrad);

public:
	// ������ͼ���ݵ��ļ�
	bool		WriteToFile(TCHAR *pathName, int nPlotIndex);
	// ������ͼĳ�������ݵ��ļ�
	bool		WriteToFile(TCHAR *pathName, int nDataIndex, int nPlotIndex);

public:
	// ������������ģʽ���򿪺����Ӧ����Ϸ�
	void		SetZoomModeBuildIn(bool zoom);
	// ����ͼ����������ģʽ��ֻ�ǻ��ƣ������ڲ�����CChartWnd�����
	void		SetZoomMode(bool zoom);
	// ����ͼ����������ʱX���������ϵ��
	void		SetZoomFactorX(double fact);
	// ����ͼ����������ʱY���������ϵ��
	void		SetZoomFactorY(double fact);
	// ����ͼ����������ʱ�������������ϵ��
	void		SetZoomFactor(double fact);
	// ����ͼ����������ʱ���������ģ����ĵĴ��ż����ļ���ʼ��ע�͵���enum
	void		SetZoomCenterMode(int center);
protected:
	// �ڲ����ú���
	RECT		GetZoomedRect(RECT destRect);

public:
	// ������ά���ߵ�����
	void		SetGridLine(bool MajorXY, bool MinorXY, bool MajorYZ, bool MinorYZ, bool MajorZX, bool MinorZX);
	// ����XY��Ļ������
	void		SetShowXYMajorGrid( bool show );
	// ����YZ��Ļ������
	void		SetShowYZMajorGrid( bool show );
	// ����ZX��Ļ������
	void		SetShowZXMajorGrid( bool show );
	// ����XY��Ļ������
	void		SetShowXYMinorGrid( bool show );
	// ����YZ��Ļ������
	void		SetShowYZMinorGrid( bool show );
	// ����ZX��Ļ������
	void		SetShowZXMinorGrid( bool show );

public:
	// �ڷֲ���ͼ�ͷ�����ͼ�У����õ�һ����ʾ�������ᣬ�ڶ�����ʾ��������
	void		SetLRAxis();
	// �ڷֲ���ͼ�У����õ�һ����ʾ�������ᣬ�ڶ�����ʾ��������
	void		SetTBAxis();
	// �ײ㺯������ȡ��ͼ�Ļ�ͼ����
	RECT		GetPlotRect(int nPlotIndex);
	// �ײ㺯������ȡ��ͼ����
	RECT		GetPlotRect();
	// �ײ㺯������ȡ������ķ�Χ
	void		GetPlotRange(double xRange[2], double yRange[2], int nPlotIndex=0);
	// �ײ㺯������ȡ���ݵķ�Χ
	void		GetDataRange(double xRange[2], double yRange[2], int nPlotIndex=0);

public:
	// ����ڲ�����ָ�룬��ָ��һ��û�ã�ֻ������ĳЩ����´��ݲ����������̺߳���
	void		*GetUserPointer();
	// �����ڲ�����ָ��
	void		SetUserPointer(void *pUser);
	// ����ڲ������־״̬��������;
	bool		GetUserFlag();
	// �����ڲ������־״̬��������;
	void		SetUserFlag(bool flag);
};

///////////////////////////////////////////////////////////////////////////////////////////
// Declaration of CChartWnd

// �����࣬��װ����Ϣ��������
class	CChart_API	CChartWnd
{
public:
	// CChartWnd�ڲ����õ����ݽṹ
	struct	CChartWndPara;
	// ����ڲ����ݵ�ָ��
	CChartWndPara *GetPara();
protected:
	// �ڲ�����
	CChartWndPara *m_pPara;
	
	// �ͷ��ڲ���Դ
	void	Release();

public:
	CChartWnd();
	virtual	~CChartWnd();

public:
	// ��ȡCChart���ָ�룬�Ե����亯��
	CChart	*GetChart(int nChartIndex=0);
	// ����CChart�������Ҫ�������໯������ֵΪԭCChart��ָ��
	CChart	*SetChart(CChart *pChart, int nChartIndex=0);
	// �����ͼ����
	virtual int		GetChartType(int nChartIndex=0);
	// �ı���ͼ����
	virtual	bool	SetChartType(int nType, int nChartIndex=0);
	// ��ô��ھ��
	HWND	GetHWnd(int nHWndIndex=0);
	// ���ô��ھ��
	void	SetHWnd(HWND hWnd, int nHWndIndex=0);
	
	// ע��ֻ�ܶ�ͬһ��hWnd Attach
	// �����Ҫ�Բ�ͬ��hWnd����Attach���뽨�����CChartWnd����

	// ճ���ڣ��������ʽ
	virtual	bool	Attach(HWND hWnd);
	// ճ���ڣ�ͬʱ���û�ͼ����
	virtual	bool	Attach(HWND hWnd, int nType);
	// ճ���ڣ�ͬʱ���û�ͼ���ͣ������ƻ�ͼ��
	virtual	bool	Attach(HWND hWnd, int nType, RECT rtClient);
	// ճ���ڣ�ճ���Ի�����, �����ؼ����
	virtual	bool	Attach(HWND hDlg, HWND hCtrl, int nType);
	// ճ���ڣ�ճ���Ի����ϣ������ؼ�ID
	virtual	bool	Attach(HWND hDlg, UINT nCtrlID, int nType);
	// ж����, ���к�����nChartIndex�������ճɹ�Attach��˳������Ϊ0, 1, 2, ...
	bool	Detach();
	bool	Detach(HWND hWnd);

	// �ػ�
	void	ReDraw();
	void	ReDraw(HWND hWnd);
	void	ReDraw(int nChartIndex);
	void	ReDraw(CChart *chart);
	
};

///////////////////////////////////////////////////////////////////////////////////////////
// CChart Ctrl
// A Standard windows control, whose class name is "ChartCtrl"
// ��ע�ắ��һ�㲻��Ҫ���ã���Ϊ��̬�������Ѿ�������
CChart_API	ATOM	RegisterChartControl();

// ��ȷ��hWnd��һ��ChartCtrl����
CChart_API	CChart	*GetChart(HWND hWnd);

// ����һ����������
CChart_API	HWND	CreatePopupChartWnd(HWND hWndParent, int nChartType, TCHAR *wndTitle, int x=0, int y=0, int cx=CW_USEDEFAULT, int cy=CW_USEDEFAULT);

// ����һ���Ӵ���
CChart_API	HWND	CreateSubChartWnd(HWND hWndParent, int nChartType, TCHAR *wndTitle, int x=0, int y=0, int cx=CW_USEDEFAULT, int cy=CW_USEDEFAULT);

///////////////////////////////////////////////////////////////////////////////////////////
// ʵ�ú���

// ���ڽ�14λʱ�䴮ת��Ϊtime_t��time_tʵ��Ϊ���ͣ�Ϊ���㣬�������������
CChart_API	double StringToTime(TCHAR *str);


///////////////////////////////////////////////////////////////////////////////////////////
// aliases
typedef	CChart		MyChart;
typedef	CChartWnd	MyChartWnd;

#ifdef CChart_API
#	undef CChart_API
#endif //PLOTDLL_API

#endif //__CHART_H_122333444455555__
///////////////////////////////////////////////////////////////////////////////////////////