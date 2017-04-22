package cn.int8.airdrum.airdrum;
import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.opengl.GLES20;
import android.opengl.GLSurfaceView;
import android.opengl.GLUtils;
import android.view.MotionEvent;

import java.io.IOException;
import java.io.InputStream;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

class MySurfaceView extends GLSurfaceView 
{
	private final float TOUCH_SCALE_FACTOR = 180.0f/320;//角度缩放比例
    private SceneRenderer mRenderer;//场景渲染器    
    
    private float mPreviousY;//上次的触控位置Y坐标
    private float mPreviousX;//上次的触控位置X坐标
    
    int textureId;//系统分配的纹理id

    MatrixState ms[] = new MatrixState[2];
	public MySurfaceView(Context context) {
        super(context);
        this.setEGLContextClientVersion(2); //设置使用OPENGL ES2.0
        mRenderer = new SceneRenderer(context);	//创建场景渲染器
        setRenderer(mRenderer);				//设置渲染器		        
        setRenderMode(GLSurfaceView.RENDERMODE_CONTINUOUSLY);//设置渲染模式为主动渲染
        ms[0] = new MatrixState();
        ms[1] = new MatrixState();

    }
	public boolean setXYZ(int index,float x,float y,float z)
	{
        mRenderer.xAngle[index] = x;//设置沿x轴旋转角度
        mRenderer.yAngle[index] = y;//设置沿y轴旋转角度
        mRenderer.zAngle[index] = z;//设置沿z轴旋转角度
        requestRender();//重绘画面
        return true;
	}
	//触摸事件回调方法
    @Override 
    public boolean onTouchEvent(MotionEvent e) 
    {
        float y = e.getY();
        float x = e.getX();
        switch (e.getAction()) {
            case MotionEvent.ACTION_MOVE:
                float dy = y - mPreviousY;//计算触控笔Y位移
                float dx = x - mPreviousX;//计算触控笔X位移
                mRenderer.xAngle[1] = x;//设置沿X轴旋转角度
                mRenderer.yAngle[1] = y;//设置沿x轴旋转角度
                mRenderer.zAngle[1]++;//设置沿z轴旋转角度
                requestRender();//重绘画面
        }
        mPreviousY = y;//记录触控笔位置
        mPreviousX = x;//记录触控笔位置
        return true;
    }

	private class SceneRenderer implements Renderer
    {
        Context mContext;
        float xAngle[];//绕X轴旋转的角度
        float yAngle[];//绕Y轴旋转的角度
    	float zAngle[]; //绕Z轴旋转的角度
    	//从指定的obj文件中加载对象
        LoadedObjectVertexNormalTexture lovo[];
        public SceneRenderer(Context context) {
            this.mContext = context;
            xAngle = new float[2];
            yAngle = new float[2];
            zAngle = new float[2];
            lovo = new LoadedObjectVertexNormalTexture[2];
        }
        public void onDrawFrame(GL10 gl)
        {
        	//清除深度缓冲与颜色缓冲
            GLES20.glClear( GLES20.GL_DEPTH_BUFFER_BIT | GLES20.GL_COLOR_BUFFER_BIT);

            //坐标系推远  
            ms[0].pushMatrix();
            ms[0].translate(-5, -3, -20);   //ch.obj
            //绕Y轴、Z轴旋转
            ms[0].rotate(xAngle[0], 1, 0, 0);
            ms[0].rotate(yAngle[0], 0, 1, 0);
            ms[0].rotate(zAngle[0], 0, 0, 1);


            //坐标系推远
            ms[1].pushMatrix();
            ms[1].translate(-5, +3, -20);   //ch.obj
            //绕Y轴、Z轴旋转
            ms[1].rotate(xAngle[1], 1, 0, 0);
            ms[1].rotate(yAngle[1], 0, 1, 0);
            ms[1].rotate(zAngle[1], 0, 0, 1);

            //若加载的物体部位空则绘制物体
            if(lovo[0]!=null)
            {
                lovo[0].drawSelf(textureId,ms[0]);
            }
            //若加载的物体部位空则绘制物体
            if(lovo[1]!=null)
            {
                lovo[1].drawSelf(textureId,ms[1]);
            }
            ms[0].popMatrix();
            ms[1].popMatrix();
        }

        public void onSurfaceChanged(GL10 gl, int width, int height) {
            //设置视窗大小及位置
        	GLES20.glViewport(0, 0, width, height);
        	//计算GLSurfaceView的宽高比
            float ratio = (float) width / height;
            //调用此方法计算产生透视投影矩阵
            ms[0].setProjectFrustum(-ratio, ratio, -1, 1, 2, 100);
            //调用此方法产生摄像机9参数位置矩阵
            ms[0].setCamera(0,0,0,0f,0f,-1f,0f,1.0f,0.0f);
            //调用此方法计算产生透视投影矩阵
            ms[1].setProjectFrustum(-ratio, ratio, -1, 1, 2, 100);
            //调用此方法产生摄像机9参数位置矩阵
            ms[1].setCamera(0,0,0,0f,0f,-1f,0f,1.0f,0.0f);
        }

        public void onSurfaceCreated(GL10 gl, EGLConfig config)
        {
            //设置屏幕背景色RGBA
            //GLES20.glClearColor(0.0f,0.0f,0.0f,1.0f);

            GLES20.glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT);

            //打开深度检测
            GLES20.glEnable(GLES20.GL_DEPTH_TEST);
            //打开背面剪裁   
            GLES20.glEnable(GLES20.GL_CULL_FACE);
            //初始化变换矩阵
            ms[0].setInitStack();
            //初始化光源位置
            ms[0].setLightLocation(0,0, 0);
            //初始化变换矩阵
            ms[1].setInitStack();
            //初始化光源位置
            ms[1].setLightLocation(0, 0, 0);
            //加载要绘制的物体
            lovo[0]=LoadUtil.loadFromFile("ch_t.obj", MySurfaceView.this.getResources(),MySurfaceView.this);
            lovo[1]=LoadUtil.loadFromFile("ch_t.obj", MySurfaceView.this.getResources(),MySurfaceView.this);
            //加载纹理
            textureId=initTexture(R.drawable.mw);
        }
    }
  	public int initTexture(int drawableId)//textureId
	{
		//生成纹理ID
		int[] textures = new int[1];
		GLES20.glGenTextures
		(
				1,          //产生的纹理id的数量
				textures,   //纹理id的数组
				0           //偏移量
		);    
		int textureId=textures[0];    
		GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, textureId);
		GLES20.glTexParameterf(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MIN_FILTER,GLES20.GL_NEAREST);
		GLES20.glTexParameterf(GLES20.GL_TEXTURE_2D,GLES20.GL_TEXTURE_MAG_FILTER,GLES20.GL_LINEAR);
		GLES20.glTexParameterf(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_WRAP_S,GLES20.GL_REPEAT);
		GLES20.glTexParameterf(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_WRAP_T,GLES20.GL_REPEAT);
        
        //通过输入流加载图片===============begin===================
        InputStream is = this.getResources().openRawResource(drawableId);
        Bitmap bitmapTmp;
        try 
        {
        	bitmapTmp = BitmapFactory.decodeStream(is);
        } 
        finally 
        {
            try 
            {
                is.close();
            } 
            catch(IOException e) 
            {
                e.printStackTrace();
            }
        }
        //通过输入流加载图片===============end===================== 
	   	GLUtils.texImage2D
	    (
	    		GLES20.GL_TEXTURE_2D, //纹理类型
	     		0, 
	     		GLUtils.getInternalFormat(bitmapTmp), 
	     		bitmapTmp, //纹理图像
	     		GLUtils.getType(bitmapTmp), 
	     		0 //纹理边框尺寸
	     );
	    bitmapTmp.recycle(); 		  //纹理加载成功后释放图片
        return textureId;
	}
}
