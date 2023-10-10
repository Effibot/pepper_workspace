package com.example.pepperdashboard.activity;

import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.widget.ImageButton;

import androidx.appcompat.app.AppCompatActivity;

import com.example.pepperdashboard.R;

public class MenuActivity extends AppCompatActivity {

    private ImageButton helpBtn; // Help Button
    private ImageButton infoBtn; // Info Button

    private ImageButton expBtn;

    private ImageButton videoBtn;

    private ImageButton misBtn;

    private ImageButton gymBtn;

    private ImageButton storyBtn;

    private ImageButton vidBtn;

    private ImageButton newsBtn;


    /**
     * The onCreate function is the first function that runs when the app starts.
     * It sets up all of the buttons and their respective functions, as well as
     * setting up a few variables to be used later in other activities.

     *
     * @param Bundle savedInstanceState Save the state of the application

     *
     * @return A void
     *
     * @docauthor Lorenzo Rossi
     */
    /**
     * The onCreate function is a function that allows the user to create an activity. In this case, when the user clicks on the button labeled &amp;quot;Experiment&amp;quot;, they will be
     * taken to another activity where they can choose which experiment they want to run. The host variable
     * is passed along so that we know what device we are connected with in order for us to send commands.

     *
     *
     * @param Bundle savedInstanceState Save the state of the application
     *
     * @return Void, so it can't be used as a return value
     *
     * @docauthor Trelent
     */
    /**
     * The onCreate function is a function that allows the user to create an activity. In this case, when the user clicks on the button labeled &amp;quot;Experiment&amp;quot;, they will be
     * taken to another activity where they can choose which experiment they want to run. The host variable
     * is passed along so that we know what device we are connected with in order for us to send commands.

     *
     *
     * @param Bundle savedInstanceState Save the state of the application
     *
     * @return Void, so it can't be used as a return value
     *
     * @docauthor Trelent
     */
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main_page);
        helpBtn = findViewById(R.id.helpBtn);
        infoBtn = findViewById(R.id.infoBtn);
        expBtn = findViewById(R.id.expBtn);
        videoBtn = findViewById(R.id.videoBtn);
        misBtn = findViewById(R.id.misBtn);
        gymBtn = findViewById(R.id.gymBtn);
        storyBtn = findViewById(R.id.storyBtn);
        vidBtn = findViewById(R.id.vidBtn);
        newsBtn = findViewById(R.id.newsBtn);

        String host = getIntent().getStringExtra("host");
        String host_pepper = getIntent().getStringExtra("host_pepper");


        helpBtn.setOnClickListener(v -> {
            Intent switchActivityIntent = new Intent(MenuActivity.this, HelpActivity.class);
            switchActivityIntent.putExtra("host", host);
            startActivity(switchActivityIntent);
        });
        infoBtn.setOnClickListener(v -> {
            Intent switchActivityIntent = new Intent(MenuActivity.this, InfoActivity.class);
            switchActivityIntent.putExtra("host", host);
            startActivity(switchActivityIntent);
        });
        expBtn.setOnClickListener(new View.OnClickListener() {
                                      /**
                                       * The onClick function is a function that allows the user to click on an object and have it perform
                                       * some action. In this case, when the user clicks on the button labeled &quot;Experiment&quot;, they will be
                                       * taken to another activity where they can choose which experiment they want to run. The host variable
                                       * is passed along so that we know what device we are connected with in order for us to send commands.

                                       *
                                       * @param View view Pass the view that was clicked to the onclick function
                                       *
                                       * @return Void, so it can't be used as a return value
                                       *
                                       * @docauthor Lorenzo Rossi
                                       */
                                      @Override
                                      public void onClick(View view) {
                                          Intent switchActivityIntent = new Intent(MenuActivity.this, ExpActivity.class);
                                          switchActivityIntent.putExtra("host", host);
                                          startActivity(switchActivityIntent);
                                      }
                                  }
        );
        videoBtn.setOnClickListener(v-> {
            Intent switchActivityIntent = new Intent(MenuActivity.this, VideoActivity.class);
            switchActivityIntent.putExtra("host", host);
            startActivity(switchActivityIntent);
        });
        misBtn.setOnClickListener(v-> {
            Intent switchActivityIntent = new Intent(MenuActivity.this, MisActivity.class);
            switchActivityIntent.putExtra("host", host);
            startActivity(switchActivityIntent);
        });
        gymBtn.setOnClickListener(v-> {
            Intent switchActivityIntent = new Intent(MenuActivity.this, GymActivity.class);
            switchActivityIntent.putExtra("host", host);
            startActivity(switchActivityIntent);
        });
        storyBtn.setOnClickListener(v-> {
            Intent switchActivityIntent = new Intent(MenuActivity.this, StoryActivity.class);
            switchActivityIntent.putExtra("host", host);
            startActivity(switchActivityIntent);
        });
        vidBtn.setOnClickListener(v-> {
            Intent switchActivityIntent = new Intent(MenuActivity.this, VidActivity.class);
            switchActivityIntent.putExtra("host", host);
            startActivity(switchActivityIntent);
        });
        newsBtn.setOnClickListener(v-> {
            Intent switchActivityIntent = new Intent(MenuActivity.this, NewsActivity.class);
            switchActivityIntent.putExtra("host", host);
            startActivity(switchActivityIntent);
        });

    }

    /**
     * The onBackPressed function is a function that overrides the default back button functionality.
     * The onBackPressed function allows us to specify what happens when the user presses the back button.
     * In this case, we want to switch activities from GameActivity to MenuActivity when they press it.

     *
     *
     * @return A switchactivityintent which is an intent that switches the activity from
     *
     * @docauthor Lorenzo Rossi
     */
    @Override
    public void onBackPressed() {
        super.onBackPressed();
        Intent switchActivityIntent = new Intent(this, MenuActivity.class);
        switchActivityIntent.putExtra("host", getIntent().getStringExtra("host"));
        startActivity(switchActivityIntent);
    }



}
