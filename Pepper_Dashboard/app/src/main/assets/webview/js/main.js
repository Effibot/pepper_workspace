function connect(data){
    let address = data;
    console.log(address);
    sessionStorage.setItem("address", address);
    //const address = sessionStorage.getItem("address");

    // Connect to ROS2 WebSocket bridge
    const ros = new ROSLIB.Ros({
            url: 'ws://'+ address+':9090' // Replace with your ROS2 WebSocket bridge URL
        });

        ros.on('connection', () => {
            console.log('Connected to ROS2 WebSocket' );
        });
        ros.on('error', (error) => {
            console.error('Error connecting to ROS2 WebSocket:', error);
        });

        ros.on('close', () => {
            console.log('Connection to ROS2 WebSocket closed');
        });

        // Define a publisher for sending numbers to a topic
        const numberTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/web_commands', // Replace with your desired topic name
            messageType: 'std_msgs/Int32' // Adjust the message type as needed
        });
        // export ros, numberTopic;
        window.ros = ros;
        window.numberTopic = numberTopic;
        // Publish a code when the navButton is clicked
        const publishButton = document.getElementById('navButton');
        publishButton.addEventListener('click', () => {
            const nav_value = 0; // Code assigned to the button
           // const message = new ROSLIB.Message({
            //    data: nav_value
           // });
           // numberTopic.publish(message);
            // edirect to address port 5000
            window.location.replace("http://"+address+":5000");
            console.log('Published:', nav_value);
        });

        // Publish a code when expButton is clicked
        const publishButton2 = document.getElementById('expButton');
        publishButton2.addEventListener('click', () => {
            const exp_value = 1; // Code assigned to the button
            const message = new ROSLIB.Message({
                data: exp_value
            });
            numberTopic.publish(message);
            console.log('Published:', exp_value);
        });

        // Publish a code when browButton is clicked
        const publishButton3 = document.getElementById('browButton');
        publishButton3.addEventListener('click', () => {
            const brow_value = 2; // Code assigned to the button
            const message = new ROSLIB.Message({
                data: brow_value
            });
            numberTopic.publish(message);
            console.log('Published:', brow_value);
        });

        // Publish a code when misButton is clicked
        const publishButton4 = document.getElementById('misButton');
        publishButton4.addEventListener('click', () => {
            // redirect to misuration.html
            window.location.assign("misuration.html");
            console.log('Redirecting to misuration page');
        });

        // Publish a code when moveButton is clicked
        const publishButton5 = document.getElementById('moveButton');
        publishButton5.addEventListener('click', () => {
            const move_value = 4; // Code assigned to the button
            const message = new ROSLIB.Message({
                data: move_value
            });
            numberTopic.publish(message);
            console.log('Published:', move_value);
        });

        // Publish a code when videoButton is clicked
        const publishButton6 = document.getElementById('videoButton');
        publishButton6.addEventListener('click', () => {
            const video_value = 5; // Code assigned to the button
            const message = new ROSLIB.Message({
                data: video_value
            });
            numberTopic.publish(message);
            console.log('Published:', video_value);
        });
    }


