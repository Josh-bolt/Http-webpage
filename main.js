/**
 * Initialize functions here.
 */
 $(document).ready(function(){
	//getUpdateStatus();
	startCANDATAInterval();
   //  updateCANdata();
}); 

/**
 * Gets CHARGER CAN values to display on the web page.
 */
 function getCANValues()
 {
     $.getJSON('/candata.json', function(data) {
         $("#battery_voltage_reading").text(data["battery_voltage"]);
         $("#battery_current_reading").text(data["battery_current"]);
         $("#charger_state_reading").text(data["charger_state"]);
         $("#bms_fault_reading").text(data["bms_fault"]);
     });
 }
 
 /**
  * Sets the interval for getting the updated DHT22 sensor values.
  */
 function startCANDATAInterval()
 {
     setInterval(getCANValues, 5000);
 }

 //document.getElementById("Mybutton").onclick = updateCANdata();
 
 function updateCANdata()
 {
    
    var xhr = new XMLHttpRequest();
    var requestURL = "/updateCAN";
    xhr.open('POST', requestURL);
    xhr.send('CAN_update_status');
    let batval = document.querySelector("#bms_val");
    batval.innerHTML = "battery val is " + Battery_vol_msb.value;
        // document.getElementById("Mybutton").onclick = function() {
        //     var response = document.getElementById("Battery_vol_msb").value;
        // // console.log("battery_msb_val",response);
        // }

}
 