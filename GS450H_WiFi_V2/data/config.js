setInterval(function ( ) {
  var xmlhttp = new XMLHttpRequest();
  var url = "/data.json";

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
      document.getElementById("oil_pump_pwm").innerHTML = response_data["oil_pump_pwm"];

      document.getElementById("voltage").innerHTML = response_data["voltage"];
      document.getElementById("water_temp").innerHTML = response_data["water_temp"];
      document.getElementById("mg1_speed").innerHTML = response_data["mg1_speed"];
      document.getElementById("mg2_speed").innerHTML = response_data["mg2_speed"];
      document.getElementById("mg1_temp").innerHTML = response_data["mg1_temp"];
      document.getElementById("mg2_temp").innerHTML = response_data["mg2_temp"];
      document.getElementById("pump_temp").innerHTML = response_data["pump_temp"];
      document.getElementById("trans_temp").innerHTML = response_data["trans_temp"];
      document.getElementById("trans_sl").innerHTML = response_data["trans_sl"];
      document.getElementById("trans_pb1").innerHTML = response_data["trans_pb1"];
      document.getElementById("trans_pb2").innerHTML = response_data["trans_pb2"];
      document.getElementById("trans_pb3").innerHTML = response_data["trans_pb3"];
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
  document.getElementById("oil_pump_pwm_set").onclick = function() {
    var val = document.getElementById("oil_pump_pwm_input").value;
    sendCommand("a"+val);
  };
  document.getElementById("save").onclick = function() {
    sendCommand("z");
  };
  document.getElementById("trans_sl_0").onclick = function() {
    sendCommand("s0");
  };
  document.getElementById("trans_sl_1").onclick = function() {
    sendCommand("s1");
  };
  document.getElementById("trans_sl_2").onclick = function() {
    sendCommand("s2");
  };
  document.getElementById("trans_sl_3").onclick = function() {
    sendCommand("s3");
  };

});

