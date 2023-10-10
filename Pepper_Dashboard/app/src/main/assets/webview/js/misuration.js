// Connect to ROS2 WebSocket bridge
const address = sessionStorage.getItem("address");

console.log(address);
const ros = new ROSLIB.Ros({
        url: 'ws://' + address + ':9090' // Replace with your ROS2 WebSocket bridge URL
});

ros.on('connection', () => {
        console.log('Connected to ROS2 WebSocket');
});

ros.on('error', (error) => {
        console.error('Error connecting to ROS2 WebSocket:', error);
});

ros.on('close', () => {
        console.log('Connection to ROS2 WebSocket closed');
});
const publishButton3 = document.getElementById('backButton');
publishButton3.addEventListener('click', () => {
        window.location.href = "./index.html";
        console.log('Redirecting to main page');
});
