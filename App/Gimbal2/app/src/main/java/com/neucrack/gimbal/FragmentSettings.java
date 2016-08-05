package com.neucrack.gimbal;

import android.content.Context;
import android.content.SharedPreferences;
import android.net.Uri;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.support.v4.app.Fragment;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.EditText;

import com.neucrack.communicate.WifiSocketManager;


/**
 * A simple {@link Fragment} subclass.
 * Activities that contain this fragment must implement the
 * {@link FragmentSettings.OnFragmentInteractionListener} interface
 * to handle interaction events.
 * Use the {@link FragmentSettings#newInstance} factory method to
 * create an instance of this fragment.
 */
public class FragmentSettings extends Fragment {
    // TODO: Rename parameter arguments, choose names that match
    // the fragment initialization parameters, e.g. ARG_ITEM_NUMBER
    private static final String ARG_PARAM1 = "param1";
    private static final String ARG_PARAM2 = "param2";

    private String mParam1;
    private String mParam2;

    private EditText mRollP,mRollI,mRollD,mPitchP,mPitchI,mPitchD,mYawP,mYawI,mYawD;
    private Button mSaveLocal,mSaveRemote,mReadLocal,mReadRemote,mCalibrateGyro,mCalibrateMag;

    private int mParamRollP,mParamRollI,mParamRollD,mParamPitchP,mParamPitchI,mParamPitchD,mParamYawP,mParamYawI,mParamYawD;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        if (getArguments() != null) {
            mParam1 = getArguments().getString(ARG_PARAM1);
            mParam2 = getArguments().getString(ARG_PARAM2);
        }
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.fragment_fragment_settings, container, false);

        mRollP = (EditText) view.findViewById(R.id.editText_PID_roll_P);
        mRollI = (EditText) view.findViewById(R.id.editText_PID_roll_I);
        mRollD = (EditText) view.findViewById(R.id.editText_PID_roll_D);
        mPitchP = (EditText) view.findViewById(R.id.editText_PID_pitch_P);
        mPitchI = (EditText) view.findViewById(R.id.editText_PID_pitch_I);
        mPitchD = (EditText) view.findViewById(R.id.editText_PID_pitch_D);
        mYawP = (EditText) view.findViewById(R.id.editText_PID_yaw_P);
        mYawI = (EditText) view.findViewById(R.id.editText_PID_yaw_I);
        mYawD = (EditText) view.findViewById(R.id.editText_PID_yaw_D);

        mSaveLocal = (Button) view.findViewById(R.id.button_save_local);
        mReadLocal = (Button) view.findViewById(R.id.button_read_local);
        mSaveRemote = (Button) view.findViewById(R.id.button_send_param);
        mReadRemote = (Button) view.findViewById(R.id.button_read);
        mCalibrateGyro = (Button) view.findViewById(R.id.button_gyro_calibrate);
        mCalibrateMag = (Button) view.findViewById(R.id.button_mag_calibrate);

        mSaveRemote.setOnClickListener(new View.OnClickListener() {
           @Override
           public void onClick(View v) {
               GetParameters();
               SendPID();
           }
       });
        mReadRemote.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                GetPIDRemote();
            }
        });
        mCalibrateGyro.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                SendCMDGyroCalibrate();
            }
        });
        mCalibrateMag.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                SendCMDMagCalibrate();
            }
        });
        mSaveLocal.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                SaveParamToLocal();
            }
        });
        mReadLocal.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                ReadParameterFromLocal();
            }
        });
        return view;
    }

    private void ReadParameterFromLocal() {
        SharedPreferences spf = getContext().getSharedPreferences("PID_Parameters",Context.MODE_PRIVATE);
        mParamRollP = spf.getInt ("rollP",0);
        mParamRollI = spf.getInt ("rollI",0);
        mParamRollD = spf.getInt ("rollD",0);
        mParamPitchP = spf.getInt("pitchP",0);
        mParamPitchI = spf.getInt("pitchI",0);
        mParamPitchD = spf.getInt("pitchD",0);
        mParamYawP = spf.getInt  ("yawP",0);
        mParamYawI = spf.getInt  ("yawI",0);
        mParamYawD = spf.getInt  ("yawD",0);
        SetParameters();
    }

    private void SaveParamToLocal() {
        SharedPreferences spf = getContext().getSharedPreferences("PID_Parameters",Context.MODE_PRIVATE);
        SharedPreferences.Editor editor = spf.edit();
        GetParameters();
        editor.putInt("rollP",mParamRollP );
        editor.putInt("rollI",mParamRollI );
        editor.putInt("rollD",mParamRollD );
        editor.putInt("pitchP",mParamPitchP);
        editor.putInt("pitchI",mParamPitchI);
        editor.putInt("pitchD",mParamPitchD);
        editor.putInt("yawP", mParamYawP);
        editor.putInt("yawI", mParamYawI);
        editor.putInt("yawD", mParamYawD);
        editor.apply();

    }

    private void SendCMDMagCalibrate() {
        byte dataToSend[] = new byte[6];
        dataToSend[0] = (byte) 0xaa;
        dataToSend[1] = (byte) 0xaf;
        dataToSend[2] = (byte) 0x01;
        dataToSend[3] = (byte) 1;
        dataToSend[4] = (byte) 4;
        int sum = 0;
        for(int i=0;i<5;i++)
        {
            sum +=dataToSend[i]&0xff;
        }
        dataToSend[5] = (byte) (sum&0xff);
        WifiSocketManager.getInstance().SendData(dataToSend);
    }

    private void SendCMDGyroCalibrate() {
        byte dataToSend[] = new byte[6];
        dataToSend[0] = (byte) 0xaa;
        dataToSend[1] = (byte) 0xaf;
        dataToSend[2] = (byte) 0x01;
        dataToSend[3] = (byte) 1;
        dataToSend[4] = (byte) 2;
        int sum = 0;
        for(int i=0;i<5;i++)
        {
            sum +=dataToSend[i]&0xff;
        }
        dataToSend[5] = (byte) (sum&0xff);
        WifiSocketManager.getInstance().SendData(dataToSend);
    }

    private void GetPIDRemote() {
        byte dataToSend[] = new byte[6];
        dataToSend[0] = (byte) 0xaa;
        dataToSend[1] = (byte) 0xaf;
        dataToSend[2] = (byte) 0x02;
        dataToSend[3] = (byte) 1;
        dataToSend[4] = (byte) 1;
        int sum = 0;
        for(int i=0;i<5;i++)
        {
            sum +=dataToSend[i]&0xff;
        }
        dataToSend[5] = (byte) (sum&0xff);
        WifiSocketManager.getInstance().SendData(dataToSend);
    }


    private void SendPID() {
        byte dataToSend[] = new byte[23];
        dataToSend[0] = (byte) 0xaa;
        dataToSend[1] = (byte) 0xaf;
        dataToSend[2] = (byte) 0x10;
        dataToSend[3] = (byte) 18;
        dataToSend[4] = (byte) ((mParamRollP&0xff00)>>8);
        dataToSend[5] = (byte) (mParamRollP&0xff);
        dataToSend[6] = (byte) ((mParamRollI&0xff00)>>8);
        dataToSend[7] = (byte) (mParamRollI&0xff);
        dataToSend[8] = (byte) ((mParamRollD&0xff00)>>8);
        dataToSend[9] = (byte) (mParamRollD&0xff);
        dataToSend[10] = (byte) ((mParamPitchP&0xff00)>>8);
        dataToSend[11] = (byte) (mParamPitchP&0xff);
        dataToSend[12] = (byte) ((mParamPitchI&0xff00)>>8);
        dataToSend[13] = (byte) (mParamPitchI&0xff);
        dataToSend[14] = (byte) ((mParamPitchD&0xff00)>>8);
        dataToSend[15] = (byte) (mParamPitchD&0xff);
        dataToSend[16] = (byte) ((mParamYawP&0xff00)>>8);
        dataToSend[17] = (byte) (mParamYawP&0xff);
        dataToSend[18] = (byte) ((mParamYawI&0xff00)>>8);
        dataToSend[19] = (byte) (mParamYawI&0xff);
        dataToSend[20] = (byte) ((mParamYawD&0xff00)>>8);
        dataToSend[21] = (byte) (mParamYawD&0xff);

        int sum = 0;
        for(int i=0;i<22;i++)
        {
            sum +=dataToSend[i]&0xff;
        }
        dataToSend[22] = (byte) (sum&0xff);
        WifiSocketManager.getInstance().SendData(dataToSend);
    }



    public void SetParameters(int rollP,int rollI,int rollD,int pitchP,int pitchI,int pitchD,int yawP,int yawI,int yawD) {
        Bundle data = new Bundle();
        data.putInt("rollP",rollP);
        data.putInt("rollI",rollI);
        data.putInt("rollD",rollD);
        data.putInt("pitchP",pitchP);
        data.putInt("pitchI",pitchI);
        data.putInt("pitchD",pitchD);
        data.putInt("yawP",yawP);
        data.putInt("yawI",yawI);
        data.putInt("yawD",yawD);
        Message msg = new Message();
        msg.setData(data);
        setMotor .sendMessage(msg);
    }

    Handler setMotor = new Handler(){
        @Override
        public void handleMessage(Message msg) {
            super.handleMessage(msg);
            Bundle data = msg.getData();
            mParamRollP = data.getInt ("rollP");
            mParamRollI = data.getInt ("rollI");
            mParamRollD = data.getInt ("rollD");
            mParamPitchP = data.getInt("pitchP");
            mParamPitchI = data.getInt("pitchI");
            mParamPitchD = data.getInt("pitchD");
            mParamYawP = data.getInt  ("yawP");
            mParamYawI = data.getInt  ("yawI");
            mParamYawD = data.getInt  ("yawD");
            SetParameters();
        }
    };

    /**
     * 将变量的参数显示到UI界面
     */
    private void SetParameters() {
        mRollP.setText(mParamRollP+"");
        mRollI.setText(mParamRollI+"");
        mRollD.setText(  mParamRollD+"");
        mPitchP.setText(mParamPitchP  +"");
        mPitchI.setText( mParamPitchI +"");
        mPitchD.setText( mParamPitchD +"");
        mYawP.setText( mParamYawP +"");
        mYawI.setText( mParamYawI +"");
        mYawD.setText(  mParamYawD+"");
    }
    /**
     * 将界面的参数保存到变量
     */
    private void GetParameters() {
        mParamRollP = 0;
        mParamRollI = 0;
        mParamRollD = 0;
        mParamPitchP = 0;
        mParamPitchI = 0;
        mParamPitchD = 0;
        mParamYawP = 0;
        mParamYawI = 0;
        mParamYawD  = 0;
        try {
            mParamRollP = Integer.parseInt(mRollP.getText().toString().trim());
            mParamRollI = Integer.parseInt(mRollI.getText().toString().trim());
            mParamRollD = Integer.parseInt(mRollD.getText().toString().trim());
            mParamPitchP = Integer.parseInt(mPitchP.getText().toString().trim());
            mParamPitchI = Integer.parseInt(mPitchI.getText().toString().trim());
            mParamPitchD = Integer.parseInt(mPitchD.getText().toString().trim());
            mParamYawP = Integer.parseInt(mYawP.getText().toString().trim());
            mParamYawI = Integer.parseInt(mYawI.getText().toString().trim());
            mParamYawD = Integer.parseInt(mYawD.getText().toString().trim());
        }catch (Exception e){

        }
    }

    public void showRawReceivedData(byte[] data) {

    }
}
