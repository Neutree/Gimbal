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
import java.net.Socket;

public class MainActivity extends FragmentActivity implements View.OnClickListener {

    private FragmentStatus mStatusPage;
    private FragmentControl mControlPage;
    private FragmentSettings mSettingsPage;

    private Button mButtonStatus,mButtonControl,mButtonSettings;
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


        //
//        new Thread(new Runnable() {
//            @Override
//            public void run() {
//                try {
//                    Log.v("a","开始连接");
//                    Socket socket = new Socket("192.168.191.1",8090);
//                    Log.v("a","..."+socket.toString()+"..." );
//                    DataOutputStream os=new DataOutputStream(socket.getOutputStream());//构建输入输出流对象
//                    DataInputStream is=new DataInputStream(socket.getInputStream());
//
//                    byte dataToRead[]=new byte[2048];
//                    while(true){
//                        int length = is.read(dataToRead);
//                        for(int i=0;i<length;++i){
//                            Log.v("a",((int)(dataToRead[i]&0xff))+"");
//                        }
//                        Log.v("a",".......................");
//                    }
//                } catch (IOException e) {
//                    e.printStackTrace();
//                }
//            }
//        }).start();
        WifiSocketManager.getInstance().init(getApplicationContext());
        WifiSocketManager.getInstance().connect(8090);
        WifiSocketManager.getInstance().setMessageCallBack(new WifiSocketManager.MesssageCallBack() {
            @Override
            public void onSendSeccess() {

            }

            @Override
            public void onSendFail() {

            }

            @Override
            public void onNewMessageCome(int[] receiveData) {
                for(int i=0;i<receiveData.length;++i)
                    Log.v("a",receiveData[i]+"");
            }
        });
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
