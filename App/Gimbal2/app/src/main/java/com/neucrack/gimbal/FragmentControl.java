package com.neucrack.gimbal;

import android.content.Context;
import android.net.Uri;
import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.SeekBar;
import android.widget.TextView;

import com.neucrack.communicate.WifiSocketManager;
import com.neucrack.component.SeekBarVertical;

import org.w3c.dom.Text;


/**
 * A simple {@link Fragment} subclass.
 * Activities that contain this fragment must implement the
 * {@link FragmentControl.OnFragmentInteractionListener} interface
 * to handle interaction events.
 * Use the {@link FragmentControl#newInstance} factory method to
 * create an instance of this fragment.
 */
public class FragmentControl extends Fragment {
    // TODO: Rename parameter arguments, choose names that match
    // the fragment initialization parameters, e.g. ARG_ITEM_NUMBER
    private static final String ARG_PARAM1 = "param1";
    private static final String ARG_PARAM2 = "param2";

    // TODO: Rename and change types of parameters
    private String mParam1;
    private String mParam2;

    private SeekBar mSeekBarRoll,mSeekBarYaw;
    private SeekBarVertical mSeekBarPitch;
    private TextView mTextViewRoll,mTextViewYaw,mTextViewPitch;
    private Button mToZero;
    private boolean mIsAllowSend = false;


    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        if (getArguments() != null) {
            mParam1 = getArguments().getString(ARG_PARAM1);
            mParam2 = getArguments().getString(ARG_PARAM2);
        }
    }

    private class onSeekBarChangeListenerImp implements SeekBar.OnSeekBarChangeListener{
        private TextView mTextView;
        private String mName;
        public onSeekBarChangeListenerImp(TextView textView,String name) {
            mTextView = textView;
            mName = name;
        }

        @Override
        public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
            //mTextView.layout();
            if(!mIsAllowSend)
                return;
            mTextView.setText(mName+"  "+ (int)(progress*3.6-180) + "°");
            byte[] dataToSend = new byte[25];
            int roll = (int)( mSeekBarRoll.getProgress()*3.6-180);
            int pitch = (int)( mSeekBarPitch.getProgress()*3.6-180);
            int yaw = (int)( mSeekBarYaw.getProgress()*3.6-180);
            dataToSend[0] = (byte) 0xaa;
            dataToSend[1] = (byte) 0xaf;
            dataToSend[2] = (byte) 0x03;
            dataToSend[3] = (byte) 20;
            dataToSend[6] = (byte) ((yaw&0xff00)>>8);
            dataToSend[7] = (byte) (yaw&0xff);
            dataToSend[8] = (byte) ((roll&0xff00)>>8);
            dataToSend[9] = (byte) (roll&0xff);
            dataToSend[10] = (byte) ((pitch&0xff00)>>8);
            dataToSend[11] = (byte) (pitch&0xff);
            int sum = 0;
            for(int i=0;i<24;i++)
            {
                sum +=dataToSend[i]&0xff;
            }
            dataToSend[24] = (byte) (sum&0xff);
            WifiSocketManager.getInstance().SendData(dataToSend);
        }

        @Override
        public void onStartTrackingTouch(SeekBar seekBar) {

        }

        @Override
        public void onStopTrackingTouch(SeekBar seekBar) {

        }
    }
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.fragment_fragment_control, container, false);
        mSeekBarRoll = (SeekBar) view.findViewById(R.id.seekBar_roll);
        mSeekBarYaw = (SeekBar) view.findViewById(R.id.seekBar_yaw);
        mSeekBarPitch = (SeekBarVertical) view.findViewById(R.id.seekbar_pitch);
        mTextViewRoll = (TextView) view.findViewById(R.id.textView_roll);
        mTextViewYaw = (TextView) view.findViewById(R.id.textView_yaw);
        mTextViewPitch = (TextView) view.findViewById(R.id.textView_pitch);
        mToZero = (Button) view.findViewById(R.id.button_to_zero);

        mSeekBarRoll.setOnSeekBarChangeListener(new onSeekBarChangeListenerImp(mTextViewRoll,"roll"));
        mSeekBarYaw.setOnSeekBarChangeListener(new onSeekBarChangeListenerImp(mTextViewYaw,"yaw"));
        mSeekBarPitch.setOnSeekBarChangeListener(new onSeekBarChangeListenerImp(mTextViewPitch,"pitch"));


        mSeekBarRoll.setProgress(50);
        mSeekBarYaw.setProgress(50);
        mSeekBarPitch.setProgress(50);

        mToZero.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                mSeekBarRoll.setProgress(50);
                mSeekBarYaw.setProgress(50);
                mSeekBarPitch.setProgress(50);
            }
        });
        return view;
    }

    @Override
    public void onResume() {
        super.onResume();
        mIsAllowSend = true;
    }

    @Override
    public void onPause() {
        super.onPause();
        mIsAllowSend = false;
    }

    public void showArm(boolean armed) {
    }
}
