function addCode(){
    var container = document.getElementById("insert_code");
    container.onkeyup = function(e) {
        var target = e.srcElement || e.target;
        var maxLength = parseInt(target.attributes["maxlength"].value, 10);
        var myLength = target.value.length;
        if (myLength >= maxLength) {
            var next = target;
            if(next.nextElementSibling==null){
                var inp = $(".code")
                var code = ""
                for(i=0;i<inp.length;i++){
                    var val = $("#"+inp[i].id).val()
                    code+=val.toString()
                }
                //connect websocket and send
                body={patient_id: code}
                console.log("The response is" + body)
                user_response_publisher.publish({data: JSON.stringify(body)})
            }
            while (next = next.nextElementSibling) {
                if (next == null){
                    break;
                }
                if (next.tagName.toLowerCase() === "input") {
                    next.focus();
                    break;
                }
            }
        }
        // Move to previous field if empty (user pressed backspace)
        else if (myLength === 0) {
            var previous = target;
            while (previous = previous.previousElementSibling) {
                if (previous == null)
                    break;
                if (previous.tagName.toLowerCase() === "input") {
                    previous.focus();
                    break;
                }
            }
        }
    }
}

