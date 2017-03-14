package cn.int8.airdrum.airdrum;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothManager;
import android.bluetooth.le.BluetoothLeScanner;
import android.content.Context;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.os.Looper;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.widget.TextView;

public class MainActivity extends AppCompatActivity {

    private TextView textView1;
    private BluetoothAdapter mBluetoothAdapter;
    private BluetoothLeScanner mBluetoothLeScanner;
    private boolean mScanning;
    private final static String TAG = MainActivity.class.getSimpleName();
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
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
        textView1=(TextView)findViewById(R.id.textView1);
    }
    @Override
    protected void onResume() {
        super.onResume();
        scanLeDevice(true);
        Log.i(TAG, "startLeScan");
    }
    @Override
    protected void onPause() {
        super.onPause();
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
    public short getShort(byte[] b, int index) {
        return (short) (((b[index + 1] << 8) | b[index + 0] & 0xff));
    }
    private BluetoothAdapter.LeScanCallback leScanCallback = new BluetoothAdapter.LeScanCallback() {
        @Override
        public void onLeScan(final BluetoothDevice device, int rssi, byte[] scanRecord) {
            short x = getShort(scanRecord,2);
            short y = getShort(scanRecord,4);
            short z = getShort(scanRecord,6);
            final String str = String.format("%d,%d,%d",x,y,z);
            if (Looper.myLooper() == Looper.getMainLooper()) {
                textView1.setText(str);
            } else {
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        textView1.setText(str);
                    }
                });
            }
        }
    };
}
