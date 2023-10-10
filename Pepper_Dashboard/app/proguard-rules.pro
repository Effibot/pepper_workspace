# Add project specific ProGuard rules here.
# You can control the set of applied configuration files using the
# proguardFiles setting in build.gradle.
#
# For more details, see
#   http://developer.android.com/guide/developing/tools/proguard.html

# If your project uses WebView with JS, uncomment the following
# and specify the fully qualified class name to the JavaScript interface
# class:
-keepclassmembers class  com.example.pepperdashboard.activity.ExpActivity{
   public *;
}
-keepclassmembers class com.example.pepperdashboard.activity.NewsActivity{
    public *;
}

# Uncomment this to preserve the line number information for
# debugging stack traces.
#-keepattributes SourceFile,LineNumberTable

# If you keep the line number information, uncomment this to
# hide the original source file name.
#-renamesourcefileattribute SourceFile

-dontwarn com.alibaba.fastjson.JSON
-dontwarn com.alibaba.fastjson.JSONArray
-dontwarn com.alibaba.fastjson.JSONObject
-dontwarn no.nordicsemi.android.dfu.DfuBaseService
-dontwarn org.bouncycastle.jsse.BCSSLParameters
-dontwarn org.bouncycastle.jsse.BCSSLSocket
-dontwarn org.bouncycastle.jsse.provider.BouncyCastleJsseProvider
-dontwarn org.conscrypt.Conscrypt$Version
-dontwarn org.conscrypt.Conscrypt
-dontwarn org.conscrypt.ConscryptHostnameVerifier
-dontwarn org.greenrobot.greendao.AbstractDao
-dontwarn org.greenrobot.greendao.AbstractDaoMaster
-dontwarn org.greenrobot.greendao.AbstractDaoSession
-dontwarn org.greenrobot.greendao.Property
-dontwarn org.greenrobot.greendao.database.Database
-dontwarn org.greenrobot.greendao.database.DatabaseOpenHelper
-dontwarn org.greenrobot.greendao.database.StandardDatabase
-dontwarn org.greenrobot.greendao.identityscope.IdentityScopeType
-dontwarn org.greenrobot.greendao.internal.DaoConfig
-dontwarn org.greenrobot.greendao.query.DeleteQuery
-dontwarn org.greenrobot.greendao.query.QueryBuilder
-dontwarn org.greenrobot.greendao.query.WhereCondition
-dontwarn org.openjsse.javax.net.ssl.SSLParameters
-dontwarn org.openjsse.javax.net.ssl.SSLSocket
-dontwarn org.openjsse.net.ssl.OpenJSSE