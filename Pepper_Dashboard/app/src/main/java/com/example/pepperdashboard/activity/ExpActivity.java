package com.example.pepperdashboard.activity;

import android.content.Intent;
import android.graphics.Bitmap;
import android.os.Bundle;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;

import androidx.appcompat.app.AppCompatActivity;

import com.example.pepperdashboard.R;

public class ExpActivity extends AppCompatActivity {
    private WebView webView;
    /**
     * The onCreate function is called when the activity is first created.
     * It sets up the webview and loads it with a url.

     *
     * @param Bundle savedInstanceState Save the state of the activity when it is closed
     *
     * @return A webview object
     *
     * @docauthor Lorenzo Rossi
     */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.esplora_activity);

        String host = getIntent().getStringExtra("host");

        webView = (WebView) findViewById(R.id.expWebView);
        webView.setWebContentsDebuggingEnabled(true);
        WebSettings webSettings = webView.getSettings();
        webSettings.setJavaScriptEnabled(true);
        webSettings.setDomStorageEnabled(true);
        webView.getSettings().setMixedContentMode(WebSettings.MIXED_CONTENT_ALWAYS_ALLOW);
        webView.loadUrl("http://"+host+":5000");
        webView.setWebViewClient(new WebViewClient() {
            /**
             * The onPageStarted function is called when the WebView begins to load a page.
             *
             *
             * @param WebView view Access the webview that is being used to display the page
             * @param String url Get the url of the page that is being loaded
             * @param Bitmap favicon Display the favicon of the website
             *
             * @return The page that is being loaded
             *
             * @docauthor Lorenzo Rossi
             */
            @Override
            public void onPageStarted(WebView view, String url, Bitmap favicon){
                super.onPageStarted(view, url, favicon);
            }
        });
    }

    @Override
    public void onBackPressed() {
        super.onBackPressed();
        Intent switchActivityIntent = new Intent(this, MenuActivity.class);
        switchActivityIntent.putExtra("host", getIntent().getStringExtra("host"));
        startActivity(switchActivityIntent);
    }
}
