package com.example.pepperdashboard.activity;

import android.content.Intent;
import android.os.Bundle;
import android.webkit.WebView;

import androidx.appcompat.app.AppCompatActivity;

import com.example.pepperdashboard.R;

public class NewsActivity extends AppCompatActivity{

    private WebView webView;

    /**
     * The onCreate function is called when the activity is first created.
     * This function sets the content view to be news_activity, and loads a webview with BBC News.

     *
     * @param Bundle savedInstanceState Save the state of the application
     *
     * @return A void
     *
     * @docauthor Lorenzo Rossi
     */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.news_activity);
        webView = (WebView) findViewById(R.id.newsWebView);
        webView.loadUrl("https://www.bbc.com/news");
    }

    /**
     * The onBackPressed function is a function that overrides the default back button functionality.
     * The default behavior of the back button is to return to the previous activity, but in this case,
     * we want it to close out of our app entirely. This function accomplishes that by calling onBackPressed()

     *
     *
     * @return Nothing
     *
     * @docauthor Lorenzo Rossi
     */
    @Override
    public void onBackPressed() {
        super.onBackPressed();
        Intent switchActivityIntent = new Intent(this, MenuActivity.class);
        startActivity(switchActivityIntent);
    }}
