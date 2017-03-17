package cn.int8.airdrum.airdrum;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothManager;
import android.bluetooth.le.BluetoothLeScanner;
import android.content.Context;
import android.content.pm.PackageManager;
import android.media.AudioManager;
import android.media.SoundPool;
import android.os.Bundle;
import android.os.Looper;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.widget.TextView;
public class MainActivity extends AppCompatActivity {

    // 操作音效
    private SoundPool sp;
    private int snare, kick, closehh, openhh;
    private MySurfaceView mGLSurfaceView;
    private TextView textView1;
    private BluetoothAdapter mBluetoothAdapter;
    private BluetoothLeScanner mBluetoothLeScanner;
    private boolean mScanning;
    private final static String TAG = MainActivity.class.getSimpleName();
    private final static boolean DEBUG = false;


    //    private final static boolean DEBUG = false;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        sp = new SoundPool(50, AudioManager.STREAM_MUSIC, 0);
        snare = sp.load(this, R.raw.snare, 1);
        kick = sp.load(this, R.raw.kick, 1);
        closehh = sp.load(this, R.raw.closehh, 1);
        openhh = sp.load(this, R.raw.openhh, 1);
        if (DEBUG) {
            setContentView(R.layout.activity_main);
        } else {
            //初始化GLSurfaceView
            mGLSurfaceView = new MySurfaceView(this);
            setContentView(mGLSurfaceView);
            mGLSurfaceView.requestFocus();//获取焦点
            mGLSurfaceView.setFocusableInTouchMode(true);//设置为可触控
        }


        if (!getPackageManager().hasSystemFeature(
                PackageManager.FEATURE_BLUETOOTH_LE)) {
            finish();
        }
        final BluetoothManager bluetoothManager = (BluetoothManager) getSystemService(Context.BLUETOOTH_SERVICE);
        mBluetoothAdapter = bluetoothManager.getAdapter();
        if (mBluetoothAdapter == null) {
            finish();
            return;
        }
        mBluetoothAdapter.enable();
        if (DEBUG) {
            textView1 = (TextView) findViewById(R.id.textView1);
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (DEBUG) {
        } else {
            mGLSurfaceView.onResume();
        }
        scanLeDevice(true);
        Log.i(TAG, "startLeScan");
    }

    @Override
    protected void onPause() {
        super.onPause();
        if (DEBUG) {
        } else {
            mGLSurfaceView.onPause();
        }
        scanLeDevice(false);
        Log.i(TAG, "stopLeScan");
    }
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        getMenuInflater().inflate(R.menu.main, menu);
        return true;
    }
        @Override
        public boolean onOptionsItemSelected(MenuItem item) {
            // Handle action bar item clicks here. The action bar will
            // automatically handle clicks on the Home/Up button, so long
            // as you specify a parent activity in AndroidManifest.xml.
            switch (item.getItemId()) {
                case R.id.restart:
                    oldCount = 65536;
                    first = true;
                    firstR2 = 0;
                    firstR0 = 0;
                    break;

                default:
                    break;
            }
            return super.onOptionsItemSelected(item);
        }
    private void scanLeDevice(final boolean enable) {
        if (enable) {
            mScanning = true;
            mBluetoothAdapter.startLeScan(leScanCallback);
        } else {
            mScanning = false;
            mBluetoothAdapter.stopLeScan(leScanCallback);
        }
    }

    private short getShort(byte[] b, int index) {
        return (short) (((b[index + 1] << 8) | b[index + 0] & 0xff));
    }

    private int getUShort(byte[] b, int index) {
        return ((int) (((b[index + 1] << 8) | b[index + 0] & 0xff))&0xffff);
    }
    //获得angleA和angleB的最小夹角（考虑到360 = 0）的问题
    private float getMinAngle(float angleA, float angleB) {
        float angle = angleA - angleB;
        if (angle > 180) {
            return 360 - angle;
        } else if (angle < -180) {
            return 360 + angle;
        } else {
            return angle;
        }
    }
    //获得以初始落点为原点，计算当前落点的位置
    //举例：如果左前右方向的初始落点firstAngle=30度，currentAngle=30的时候，得到值为0，currentAngle=40的时候，得到值为10，currentAngle=20时，得到值为350
    private float getAngle(float currentAngle,float firstAngle) {
        float angle = currentAngle - firstAngle;
        if (angle < 0) {
            return 360 + angle;
        } else {
            return angle;
        }
    }

    float oldCount = 65536;
    boolean first = true;
    float firstR2 = 0;
    float firstR0 = 0;
    private BluetoothAdapter.LeScanCallback leScanCallback = new BluetoothAdapter.LeScanCallback() {
        @Override
        public void onLeScan(final BluetoothDevice device, int rssi, byte[] scanRecord) {
//            final float x = 360 -  (getShort(scanRecord,2) + 10000) /20000.00f * 360;
//            final float y =360 -  (getShort(scanRecord,4) ) / 100.00f;
//            final float z =360 - (getShort(scanRecord,6) + 10000) /20000.00f * 360;
            final float count = getShort(scanRecord, 2);
            final float r2 = 180 + getShort(scanRecord, 4) / 100.00f;//r2的范围是-180~+180 加180后得到0~360数据
            final float r0 = getUShort(scanRecord, 6) / 100.00f;//r0的范围是0~360


            System.out.println("r2:\t"+ r2+"\tr0:\t"+ r0 + "\tangle:" + getAngle(r0,firstR0)+"\tfirstr0:"+firstR0);
            if (count > oldCount) {
                //第一次落棒
                if (first) {
                    first = false;
                    //第一次落棒时保存第一次落棒的位置
                    firstR2 = r2;
                    firstR0 = r0;
                } else {
                    float angleR0 = getMinAngle(r0,firstR0);
                    float angleR2 = getMinAngle(r2,firstR2);
                    //右鼓
                    if (angleR0 > 15) {
                        //上
                        if (angleR2 > 25) {
                            sp.play(closehh, 1.0f, 0.3f, 0, 0, 1.0f);
                        } else {//下
                            sp.play(snare, 1.0f, 0.3f, 0, 0, 1.0f);
                        }
                    } else if (angleR0 < -15) {//左鼓
                        //上
                        if (angleR2 > 25) {
                            sp.play(closehh, 1.0f, 0.3f, 0, 0, 1.0f);
                        } else {//下
                            sp.play(snare, 1.0f, 0.3f, 0, 0, 1.0f);
                        }
                    } else {//中鼓
                        //上
                        if (angleR2 > 25) {
                            sp.play(openhh, 1.0f, 0.3f, 0, 0, 1.0f);
                        } else {//下
                            sp.play(kick, 1.0f, 0.3f, 0, 0, 1.0f);
                        }
                    }
                }
            }
            oldCount = count;
            if (Looper.myLooper() == Looper.getMainLooper()) {
                if (DEBUG) {
                    final String str = String.format("%.2f          %.2f          %.2f", count, r2, r0);
                    textView1.setText(str);
                } else {
                    mGLSurfaceView.setXYZ(0, r0 < 180 ? -r2:r2, getAngle(r0,firstR0));
                }
            } else {
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        if (DEBUG) {
                            final String str = String.format("%.2f          %.2f          %.2f", count, r2, r0);
                            textView1.setText(str);
                        } else {
                            mGLSurfaceView.setXYZ(0, r0 < 180 ? -r2:r2, getAngle(r0,firstR0));
                        }
                    }
                });
            }
        }
    };
}

