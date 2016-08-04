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

    private Button mButtonStatus,mButtonControl,mButtonSettings;

    private byte mReceivedBuffer[] = new byte[2048];
    private int mReceivedBufferIndex = -1;

    private int mStatusConnectedToServer = -1;//-1：未连接 0：连接成功 1：正在连接 2;连接断开

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        mButtonStatus = (Button) findViewById(R.id.button_status);
        mButtonControl = (Button) findViewById(R.id.button_control);
        mButtonSettings = (Button) findViewById(R.id.button_settings);
        mButtonStatus.setOnClickListener(this);
        mButtonControl.setOnClickListener(this);
        mButtonSettings.setOnClickListener(this);

        //初始页面
        FragmentManager fm = getSupportFragmentManager();
        FragmentTransaction transaction = fm.beginTransaction();
        mStatusPage = new FragmentStatus();
        transaction.add(R.id.fragment_content,mStatusPage);
        transaction.commit();

        WifiSocketManager.getInstance().init(getApplicationContext());
        //建立一个线程专门用来监控连接
        new Thread(new Runnable() {
            @Override
            public void run() {
                while(true){
                    if(mStatusConnectedToServer == -1) {//未连接
                        mStatusConnectedToServer = 1;//标志正在连接
                        showHintConnecting(mStatusConnectedToServer);
                        WifiSocketManager.getInstance().connect(8090);
                    }
                    try {
                        Thread.sleep(5000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }

            }
        }).start();
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
                                dealMessageFrame(mReceivedBuffer);
                                //将已经处理过的帧从缓冲区删除
                                for (int j = 0; j < mReceivedBufferIndex + 1 - frameLength; ++j)
                                    mReceivedBuffer[j] = mReceivedBuffer[frameLength + j];
                                mReceivedBufferIndex -= frameLength;
                                i = 0;
                            } else//长度不够
                                return;
                        }
                    }
                }
            }
            @Override
            public void onNewMessageCome(byte[] receiveData) {
                //将数据放入缓冲区
                for(int i=0;i<receiveData.length;++i)
                    mReceivedBuffer[++mReceivedBufferIndex] = receiveData[i];
                deal();
            }
        });
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
        for(int i=0;i<contentLength +5;++i){
            Log.v("a",(messageFrame[i]&0xff)+"");
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
                break;
            case R.id.button_control:
                if(mControlPage==null)
                    mControlPage = new FragmentControl();
                transaction.replace(R.id.fragment_content,mControlPage);
                break;
            case R.id.button_settings:
                if(mSettingsPage==null)
                    mSettingsPage = new FragmentSettings();
                transaction.replace(R.id.fragment_content,mSettingsPage);
                break;
            default:
                break;
        }
        transaction.commit();
    }
}