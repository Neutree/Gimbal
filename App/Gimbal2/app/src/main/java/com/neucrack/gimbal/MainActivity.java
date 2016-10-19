package com.neucrack.gimbal;

import android.content.DialogInterface;
import android.net.Uri;
import android.support.v4.app.FragmentActivity;
import android.support.v4.app.FragmentManager;
import android.support.v4.app.FragmentTransaction;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import com.neucrack.communicate.WifiSocketManager;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.lang.reflect.Array;
import java.net.Socket;

public class MainActivity extends FragmentActivity implements View.OnClickListener {

    private FragmentStatus mStatusPage;
    private FragmentControl mControlPage;
    private FragmentSettings mSettingsPage;

    private TextView mButtonStatus,mButtonControl,mButtonSettings;

    private byte mReceivedBuffer[] = new byte[2048];
    private int mReceivedBufferIndex = -1;

    private int mStatusConnectedToServer = -1;//-1：未连接 0：连接成功 1：正在连接 2;连接断开
    private Thread mConnectionThread;
    private boolean mIsShowRawReceivedData=false;
    private boolean mIsProgramExit = false;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        Log.v("a","onCreate");
        setContentView(R.layout.activity_main);

        mButtonStatus = (TextView) findViewById(R.id.button_status);
        mButtonControl = (TextView) findViewById(R.id.button_control);
        mButtonSettings = (TextView) findViewById(R.id.button_settings);
        mButtonStatus.setOnClickListener(this);
        mButtonControl.setOnClickListener(this);
        mButtonSettings.setOnClickListener(this);

        //初始页面
        FragmentManager fm = getSupportFragmentManager();
        FragmentTransaction transaction = fm.beginTransaction();
        mStatusPage = new FragmentStatus();
        mControlPage = new FragmentControl();
        mSettingsPage  = new FragmentSettings();
        transaction.replace(R.id.fragment_content, mStatusPage);
        transaction.commit();

        WifiSocketManager.getInstance().init(getApplicationContext());
        mIsProgramExit = false;
        if(mConnectionThread == null) {
            //建立一个线程专门用来监控连接
            mConnectionThread = new Thread(new ConnectionDetectThread());
            mConnectionThread.start();
        }
        WifiSocketManager.getInstance().setConnectionCallBack(new WifiSocketManager.ConnectionCallBack() {
            @Override
            public void onConnected() {
                Log.v("a","建立了连接");
                mStatusConnectedToServer = 0;
                showHintConnecting(mStatusConnectedToServer);
            }

            @Override
            public void onDisConnected() {
                if(mStatusConnectedToServer == 0){
                    Log.v("a","连接断开");
                    mStatusConnectedToServer = 2;//刚刚断开连接
                    showHintConnecting(mStatusConnectedToServer);
                    mStatusConnectedToServer = -1;
                    return;
                }
                Log.v("a","建立连接失败");
                mStatusConnectedToServer = -1;
                showHintConnecting(mStatusConnectedToServer);
            }
        });
        WifiSocketManager.getInstance().setMessageCallBack(new WifiSocketManager.MesssageCallBack() {
            @Override
            public void onSendSeccess() {

            }

            @Override
            public void onSendFail() {

            }
            public void deal(){
                //查找有效数据帧
                if(mReceivedBufferIndex>=4) {//数据长度够组成一帧数据
                    for (int i = 0; i < mReceivedBufferIndex; ++i) {
                        if ((mReceivedBuffer[i] & 0xff) == 0xaa && (mReceivedBuffer[i + 1] & 0xff) == 0xaa) {//帧头：0xaaaa
                            //丢弃开头无效数据
                            if (i > 0)
                                for (int j = 0; j < mReceivedBufferIndex + 1 - i; ++j)
                                    mReceivedBuffer[j] = mReceivedBuffer[i + j];
                            mReceivedBufferIndex -= i;
                            if (mReceivedBufferIndex < 4)
                                return;
                            //判断长度，一帧数据是否全部到达完毕
                            int frameLength = (mReceivedBuffer[3] & 0xff) + 5;
                            if (frameLength <= mReceivedBufferIndex + 1) {//长度够
                                //校验帧
                                int sum=0;
                                for(int j=0;j<frameLength-1;++j)
                                    sum+=(mReceivedBuffer[j]&0xff);
                                sum&=0xff;
                                if(sum == (mReceivedBuffer[frameLength-1]&0xff))
                                    dealMessageFrame(mReceivedBuffer);
                                //将已经处理过的帧从缓冲区删除
                                for (int j = 0; j < mReceivedBufferIndex + 1 - frameLength; ++j)
                                    mReceivedBuffer[j] = mReceivedBuffer[frameLength + j];
                                mReceivedBufferIndex -= frameLength;
                                i = -1;
                            } else//长度不够
                                return;
                        }
                    }
                }
            }
            @Override
            public void onNewMessageCome(byte[] receiveData) {
                if(mIsShowRawReceivedData)
                    showRawReceivedData(receiveData);
                //将数据放入缓冲区
                for(int i=0;i<receiveData.length&&(mReceivedBufferIndex<2048);++i)
                    mReceivedBuffer[++mReceivedBufferIndex] = receiveData[i];
                deal();
            }
        });
    }

    class ConnectionDetectThread implements Runnable{

        @Override
        public void run() {
            while (!mIsProgramExit) {
                if (mStatusConnectedToServer == -1) {//未连接
                    mStatusConnectedToServer = 1;//标志正在连接
                    showHintConnecting(mStatusConnectedToServer);
                    WifiSocketManager.getInstance().connect(8080);
                }
                try {
                    Thread.sleep(5000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }
    private void showRawReceivedData(byte[] data) {
        mSettingsPage.showRawReceivedData(data);
    }

    private void showHintConnecting(final int connectionStatus) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                if(connectionStatus == -1)
                    Toast.makeText(getApplicationContext(),"连接服务器失败",Toast.LENGTH_SHORT).show();
                else if(connectionStatus == 1)
                    Toast.makeText(getApplicationContext(),"正在连接服务器",Toast.LENGTH_SHORT).show();
                else if(connectionStatus == 0)
                    Toast.makeText(getApplicationContext(),"连接服务器成功",Toast.LENGTH_SHORT).show();
                else if(connectionStatus == 2)
                    Toast.makeText(getApplicationContext(),"连接断开",Toast.LENGTH_SHORT).show();
            }
        });
    }

    public void dealMessageFrame(byte[] messageFrame) {
        int contentLength = (messageFrame[3]&0xff);
        switch ((messageFrame[2]&0xff)){
            case 1://姿态数据
                if(contentLength>=6)
                    mStatusPage.showAngle( ((messageFrame[4])<<8|(messageFrame[5]&0xff))/100.0, ((messageFrame[6])<<8|(messageFrame[7]&0xff))/100.0, ((messageFrame[8])<<8|(messageFrame[9]&0xff))/100.0 );
//                if(contentLength>=12)
//                    mControlPage.showArm(((messageFrame[11]&0xff)>0?true:false));
                break;
            case 2://传感器数据
                if(contentLength>=18) {
                    double ax,ay,az;
                    ax =((messageFrame[4]) << 8 | (messageFrame[5] & 0xff)) / 100.0;
                    ay =((messageFrame[6]) << 8 | (messageFrame[7] & 0xff)) / 100.0;
                    az =((messageFrame[8]) << 8 | (messageFrame[9] & 0xff)) / 100.0;
                    if(ax==0&&ay==0&&az==0)
                        Log.v("b",".....acc 0,0,0");
                    mStatusPage.showRawData(ax,ay,az,
                            ((messageFrame[10]) << 8 | (messageFrame[11] & 0xff)), ((messageFrame[12]) << 8 | (messageFrame[13] & 0xff)), ((messageFrame[14]) << 8 | (messageFrame[15] & 0xff)),
                            ((messageFrame[16]) << 8 | (messageFrame[17] & 0xff)), ((messageFrame[18]) << 8 | (messageFrame[19] & 0xff)), ((messageFrame[20]) << 8 | (messageFrame[21] & 0xff)));
                }
                break;
            case 3://遥控数据
                if(contentLength>=8)
                    mStatusPage.showRCData(  (((messageFrame[8]&0xff)<<8|(messageFrame[9]&0xff))-1000)/2.77778 - 180, (((messageFrame[10]&0xff)<<8|(messageFrame[11]&0xff))-1000)/2.77778 - 180,(((messageFrame[6]&0xff)<<8|(messageFrame[7]&0xff))-1000)/2.77778 - 180);
                break;
            case 5://电压数据
                if(contentLength>=2)
                    mStatusPage.showVoltage(((messageFrame[4])<<8|(messageFrame[5]&0xff))/100.0);
                break;
            case 6://电机数据

                if(contentLength>=6) {
                    mStatusPage.showMotor(((messageFrame[4]) << 8 | (messageFrame[5] & 0xff)), ((messageFrame[6]) << 8 | (messageFrame[7] & 0xff)), ((messageFrame[8]) << 8 | (messageFrame[9] & 0xff)));
                }
                break;
            case 0x10://PID参数
                mSettingsPage.SetParameters(((messageFrame[4]&0xff)<<8|(messageFrame[5]&0xff)), ((messageFrame[6]&0xff)<<8|(messageFrame[7]&0xff)), ((messageFrame[8]&0xff)<<8|(messageFrame[9]&0xff)),
                                            ((messageFrame[10]&0xff)<<8|(messageFrame[11]&0xff)), ((messageFrame[12]&0xff)<<8|(messageFrame[13]&0xff)), ((messageFrame[14]&0xff)<<8|(messageFrame[15]&0xff)),
                                            ((messageFrame[16]&0xff)<<8|(messageFrame[17]&0xff)), ((messageFrame[18]&0xff)<<8|(messageFrame[19]&0xff)), ((messageFrame[20]&0xff)<<8|(messageFrame[21]&0xff)));
                 break;
        }
    }

    @Override
    public void onClick(View v) {
        FragmentManager fm = getSupportFragmentManager();
        FragmentTransaction transaction = fm.beginTransaction();
        switch (v.getId()){
            case R.id.button_status:
                if(mStatusPage==null)
                    mStatusPage = new FragmentStatus();
                transaction.replace(R.id.fragment_content,mStatusPage);
                mButtonStatus.setTextSize(18);
                mButtonControl.setTextSize(14);
                mButtonSettings.setTextSize(14);
                break;
            case R.id.button_control:
                if(mControlPage==null)
                    mControlPage = new FragmentControl();
                transaction.replace(R.id.fragment_content,mControlPage);
                mButtonStatus.setTextSize(14);
                mButtonControl.setTextSize(18);
                mButtonSettings.setTextSize(14);
                break;
            case R.id.button_settings:
                if(mSettingsPage==null)
                    mSettingsPage = new FragmentSettings();
                transaction.replace(R.id.fragment_content,mSettingsPage);
                mButtonStatus.setTextSize(14);
                mButtonControl.setTextSize(14);
                mButtonSettings.setTextSize(18);
                break;
            default:
                break;
        }
        transaction.commit();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        Log.v("a","onDestroy");
        mIsProgramExit = true;
        mConnectionThread = null;
    }
}
