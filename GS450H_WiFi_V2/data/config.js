setInterval(function ( ) {
  var xmlhttp = new XMLHttpRequest();
  var url = "/config.json";

  xmlhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      var response_data = JSON.parse(this.responseText);
      document.getElementById("pedal_min").innerHTML = response_data["pedal_min"];
      document.getElementById("pedal_max").innerHTML = response_data["pedal_max"];
      document.getElementById("max_torque_fwd").innerHTML = response_data["max_torque_fwd"];
      document.getElementById("max_torque_rev").innerHTML = response_data["max_torque_rev"];
      document.getElementById("regen_factor").innerHTML = response_data["regen_factor"];
      document.getElementById("regen_limit").innerHTML = response_data["regen_limit"];
      document.getElementById("throttle_exp").innerHTML = response_data["throttle_exp"];
      document.getElementById("precharge_voltage").innerHTML = response_data["precharge_voltage"];
    }
  };
  xmlhttp.open("GET", url, true);
  xmlhttp.send();
}, 500 ) ;

function sendCommand(command) {
  var xmlhttp = new XMLHttpRequest();
  xmlhttp.open("POST", "/command", true);
  xmlhttp.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
  xmlhttp.send("command="+command);
}

document.addEventListener("DOMContentLoaded", function(event) {
  document.getElementById("pedal_min_set").onclick = function() {
    sendCommand("e");
  };
  document.getElementById("pedal_max_set").onclick = function() {
    sendCommand("r");
  };
  document.getElementById("max_torque_fwd_set").onclick = function() {
    var val = document.getElementById("max_torque_fwd_input").value;
    sendCommand("t"+val);
  };
  document.getElementById("max_torque_rev_set").onclick = function() {
    var val = document.getElementById("max_torque_rev_input").value;
    sendCommand("y"+val);
  };
  document.getElementById("regen_factor_set").onclick = function() {
    var val = document.getElementById("regen_factor_input").value;
    sendCommand("u"+val);
  };
  document.getElementById("regen_limit_set").onclick = function() {
    var val = document.getElementById("regen_limit_input").value;
    sendCommand("i"+val);
  };
  document.getElementById("throttle_exp_set_linear").onclick = function() {
    sendCommand("o");
  };
  document.getElementById("throttle_exp_set_exponential").onclick = function() {
    sendCommand("p");
  };
  document.getElementById("precharge_voltage_set").onclick = function() {
    var val = document.getElementById("precharge_voltage_input").value;
    sendCommand("v"+val);
  };
  document.getElementById("save").onclick = function() {
    sendCommand("z");
  };
});

