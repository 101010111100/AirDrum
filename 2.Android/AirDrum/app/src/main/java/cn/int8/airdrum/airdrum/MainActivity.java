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
    private final static boolean DEBUG = true;
    ;

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

    //获得angleA和angleB的最小夹角（考虑到360 = 0）的问题
    private float getAngle(float angleA, float angleB) {
        float angle = angleA - angleB;
        if (angle > 180) {
            return 360 - angle;
        } else if (angle < -180) {
            return 360 + angle;
        } else {
            return angle;
        }
    }

    float oldCount = 65536;
    boolean first = false;
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
            final float r0 = getShort(scanRecord, 6) / 100.00f;//r0的范围是0~360
            final boolean play;
            if (count > oldCount) {
                //第一次落棒
                if (first) {
                    first = false;
                    //第一次落棒时保存第一次落棒的位置
                    firstR2 = r2;
                    firstR0 = r0;
                } else {
                    float angleR0 = getAngle(firstR0, r0);
                    float angleR2 = getAngle(firstR2, r2);
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
                oldCount = count;
                if (Looper.myLooper() == Looper.getMainLooper()) {
                    if (DEBUG) {
                        final String str = String.format("%.2f          %.2f          %.2f", count, r2, r0);
                        textView1.setText(str);
                    } else {
                        mGLSurfaceView.setXYZ(0, r2, r0);
                    }
                } else {
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            if (DEBUG) {
                                final String str = String.format("%.2f          %.2f          %.2f", count, r2, r0);
                                textView1.setText(str);
                            } else {
                                mGLSurfaceView.setXYZ(0, r2, r0);
                            }
                        }
                    });
                }
            }
        }
    };
}

