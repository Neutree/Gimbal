package com.neucrack.gimbal;

import android.content.Context;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.support.v4.app.Fragment;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;

import com.neucrack.communicate.WifiSocketManager;


/**
 * A simple {@link Fragment} subclass.
 * Activities that contain this fragment must implement the
 * {@link FragmentWiFiSettings.OnFragmentInteractionListener} interface
 * to handle interaction events.
 * Use the {@link FragmentWiFiSettings#newInstance} factory method to
 * create an instance of this fragment.
 */
public class FragmentWiFiSettings extends Fragment {

    private WebView mWebView;
    private TextView mUrl;
    private Button openWiFiSettingsWeb;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        if (getArguments() != null) {
        }
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        final View view_ = inflater.inflate(R.layout.fragment_wifi_settings, container, false);
        openWiFiSettingsPage(view_);
        openWiFiSettingsWeb = (Button) view_.findViewById(R.id.button_wifi_settings_open);
        openWiFiSettingsWeb.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                String url="http://"+mUrl.getText().toString().trim();
                mWebView.loadUrl(url);
            }
        });
        return view_;
    }
    private void openWiFiSettingsPage(View view){
        mWebView = (WebView) view.findViewById(R.id.webview_wifi_settings);
        mUrl = (EditText)view.findViewById(R.id.webview_addr);
        String url="http://"+mUrl.getText().toString().trim();
        //WebView加载web资源
        mWebView.loadUrl(url);
        //覆盖WebView默认使用第三方或系统默认浏览器打开网页的行为，使网页用WebView打开
        mWebView.setWebViewClient(new WebViewClient(){
            @Override
            public boolean shouldOverrideUrlLoading(WebView view, String url) {
                // TODO Auto-generated method stub
                //返回值是true的时候控制去WebView打开，为false调用系统浏览器或第三方浏览器
                view.loadUrl(url);
                return true;
            }
        });
    }
}
