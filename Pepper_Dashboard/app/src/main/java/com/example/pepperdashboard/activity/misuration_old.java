package com.example.pepperdashboard.activity;

import androidx.appcompat.app.AppCompatActivity;

import android.graphics.Bitmap;
import android.os.Bundle;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;

import com.example.pepperdashboard.R;

public class misuration_old extends AppCompatActivity {
    private WebView myWebView;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        String host = getIntent().getStringExtra("host");
        String host_pepper = getIntent().getStringExtra("host_pepper");

        myWebView = (WebView) findViewById(R.id.webview);
        myWebView.setWebContentsDebuggingEnabled(true);
        WebSettings webSettings = myWebView.getSettings();
        webSettings.setJavaScriptEnabled(true);
        webSettings.setDomStorageEnabled(true);
        myWebView.getSettings().setMixedContentMode(WebSettings.MIXED_CONTENT_ALWAYS_ALLOW);
        myWebView.loadUrl("file:///android_asset/index.html");
        myWebView.setWebViewClient(new WebViewClient() {
            @Override
            public void onPageStarted(WebView view, String url, Bitmap favicon){
                super.onPageStarted(view, url, favicon);
            }
            @Override
            public void onPageFinished(WebView view, String url){
                super.onPageFinished(view, url);
                if(url != null && url.equals("file:///android_asset/index.html"))
                    myWebView.evaluateJavascript("javascript:connect(\"" + host + "\")", null);

            }
        });

    }

    @Override
    public void onBackPressed(){
        if(myWebView != null && myWebView.canGoBack()) {
            myWebView.loadUrl("file:///android_asset/index.html");
        }
        else{
            super.onBackPressed();
        }
    }

}