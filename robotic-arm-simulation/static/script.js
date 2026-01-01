// In your script.js file
function sendCommand() {
    const command = document.getElementById('commandInput').value;

    fetch('/execute_command', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ command: command })
    })
    .then(response => {
        // Check if the response is valid before parsing
        if (!response.ok) {
            throw new Error('Network response was not ok');
        }
        return response.json();
    })
    .then(data => {
        const responseElement = document.getElementById('response');
        if (data.result) {
            // Display the success message
            responseElement.innerText = `Server: ${data.result}`;
            responseElement.style.color = 'green';
        } else if (data.error) {
            // Display the error message
            responseElement.innerText = `Error: ${data.error}`;
            responseElement.style.color = 'red';
        }
    })
    .catch(error => {
        console.error('Error:', error);
        document.getElementById('response').innerText = `Error: An unexpected error occurred.`;
        document.getElementById('response').style.color = 'red';
    });
}