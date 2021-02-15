var lang =['it-IT', 'Italia']
var recognition;
var recognizing = false;
var final_transcript = '';
var ignore_onend;
var start_timestamp;
var state="IDLE"


function record(){
    console.log("Record")
    var start_button = $("#start_button")
    var start_img = document.getElementById("start_img")
    start_button.on("click", function(event){
        startButton(event, this)
    })
    
    if (!('webkitSpeechRecognition' in window)) {
      upgrade();
    } else {
      recognition = new webkitSpeechRecognition();
      recognition.continuous = true;
      recognition.interimResults = true;
      recognition.onstart = function() {
        recognizing = true;
        showInfo('info_speak_now');
        start_img.src = 'data:image/svg+xml,%3Csvg%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20viewBox%3D%220%200%2024%2024%22%3E%3Cpath%20fill%3D%22none%22%20d%3D%22M0%200h24v24H0V0z%22%2F%3E%3Cpath%20d%3D%22M9%2013c2.21%200%204-1.79%204-4s-1.79-4-4-4-4%201.79-4%204%201.79%204%204%204zm0-6c1.1%200%202%20.9%202%202s-.9%202-2%202-2-.9-2-2%20.9-2%202-2zm0%208c-2.67%200-8%201.34-8%204v2h16v-2c0-2.66-5.33-4-8-4zm-6%204c.22-.72%203.31-2%206-2%202.7%200%205.8%201.29%206%202H3zM15.08%207.05c.84%201.18.84%202.71%200%203.89l1.68%201.69c2.02-2.02%202.02-5.07%200-7.27l-1.68%201.69zM20.07%202l-1.63%201.63c2.77%203.02%202.77%207.56%200%2010.74L20.07%2016c3.9-3.89%203.91-9.95%200-14z%22%2F%3E%3C%2Fsvg%3E';
      };
      recognition.onerror = function(event) {
        if (event.error == 'no-speech') {
          start_img.src = 'data:image/svg+xml,%3Csvg%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20viewBox%3D%220%200%2024%2024%22%3E%3Cpath%20fill%3D%22none%22%20d%3D%22M0%200h24v24H0V0z%22%2F%3E%3Cpath%20d%3D%22M12%206v3l4-4-4-4v3c-4.42%200-8%203.58-8%208%200%201.57.46%203.03%201.24%204.26L6.7%2014.8c-.45-.83-.7-1.79-.7-2.8%200-3.31%202.69-6%206-6zm6.76%201.74L17.3%209.2c.44.84.7%201.79.7%202.8%200%203.31-2.69%206-6%206v-3l-4%204%204%204v-3c4.42%200%208-3.58%208-8%200-1.57-.46-3.03-1.24-4.26z%22%2F%3E%3C%2Fsvg%3E';
          showInfo('info_no_speech');
          ignore_onend = true;
        }
        if (event.error == 'audio-capture') {
          start_img.src = 'data:image/svg+xml,%3Csvg%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20width%3D%2224%22%20height%3D%2224%22%20viewBox%3D%220%200%2024%2024%22%3E%3Cpath%20fill%3D%22none%22%20d%3D%22M0%200h24v24H0V0z%22%2F%3E%3Cpath%20d%3D%22M12%2015c1.66%200%202.99-1.34%202.99-3L15%206c0-1.66-1.34-3-3-3S9%204.34%209%206v6c0%201.66%201.34%203%203%203zm-1.2-9.1c0-.66.54-1.2%201.2-1.2s1.2.54%201.2%201.2l-.01%206.2c0%20.66-.53%201.2-1.19%201.2s-1.2-.54-1.2-1.2V5.9zm6.5%206.1c0%203-2.54%205.1-5.3%205.1S6.7%2015%206.7%2012H5c0%203.41%202.72%206.23%206%206.72V22h2v-3.28c3.28-.48%206-3.3%206-6.72h-1.7z%22%2F%3E%3C%2Fsvg%3E';
          showInfo('info_no_microphone');
          ignore_onend = true;
        }
        if (event.error == 'not-allowed') {
          if (event.timeStamp - start_timestamp < 100) {
            showInfo('info_blocked');
          } else {
            showInfo('info_denied');
          }
          ignore_onend = true;
        }
      };
      recognition.onend = function() {
        if(state=="SPEAK"){
          recognition.start();
          if (ignore_onend) {
            return;
          }  
        }else{
          recognizing=false
          start_img.src ='data:image/svg+xml,%3Csvg%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20viewBox%3D%220%200%2024%2024%22%3E%3Cpath%20fill%3D%22none%22%20d%3D%22M0%200h24v24H0V0z%22%2F%3E%3Cpath%20d%3D%22M12%2015c1.66%200%202.99-1.34%202.99-3L15%206c0-1.66-1.34-3-3-3S9%204.34%209%206v6c0%201.66%201.34%203%203%203zm-1.2-9.1c0-.66.54-1.2%201.2-1.2s1.2.54%201.2%201.2l-.01%206.2c0%20.66-.53%201.2-1.19%201.2s-1.2-.54-1.2-1.2V5.9zm6.5%206.1c0%203-2.54%205.1-5.3%205.1S6.7%2015%206.7%2012H5c0%203.41%202.72%206.23%206%206.72V22h2v-3.28c3.28-.48%206-3.3%206-6.72h-1.7z%22%2F%3E%3C%2Fsvg%3E';
        }
        
        /*start_img.src ='data:image/svg+xml,%3Csvg%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20viewBox%3D%220%200%2024%2024%22%3E%3Cpath%20fill%3D%22none%22%20d%3D%22M0%200h24v24H0V0z%22%2F%3E%3Cpath%20d%3D%22M12%2015c1.66%200%202.99-1.34%202.99-3L15%206c0-1.66-1.34-3-3-3S9%204.34%209%206v6c0%201.66%201.34%203%203%203zm-1.2-9.1c0-.66.54-1.2%201.2-1.2s1.2.54%201.2%201.2l-.01%206.2c0%20.66-.53%201.2-1.19%201.2s-1.2-.54-1.2-1.2V5.9zm6.5%206.1c0%203-2.54%205.1-5.3%205.1S6.7%2015%206.7%2012H5c0%203.41%202.72%206.23%206%206.72V22h2v-3.28c3.28-.48%206-3.3%206-6.72h-1.7z%22%2F%3E%3C%2Fsvg%3E';
        if (!final_transcript) {
          showInfo('info_start');
          return;
        }*/
        showInfo('');
        /*if (window.getSelection) {
          window.getSelection().removeAllRanges();
          var range = document.createRange();
          range.selectNode(document.getElementById('final_span'));
          window.getSelection().addRange(range);
        }*/
      };
      recognition.onresult = function(event) {
        var interim_transcript = '';
        for (var i = event.resultIndex; i < event.results.length; ++i) {
          if (event.results[i].isFinal) {
            final_transcript = event.results[i][0].transcript;
          } else {
            interim_transcript += event.results[i][0].transcript;
          }
        }
      };
    }
}


function upgrade() {
  start_button.style.visibility = 'hidden';
  showInfo('info_upgrade');
}

function startButton(event) {
    console.log("Start Button clicked")
  if (recognizing) {
    state="STOP"
    recognition.stop();
    showInfo(final_transcript)
    if (final_transcript!=""){
      final_transcript = final_transcript.replace(/[^a-zA-Z ]/g, "")
      final_transcript = final_transcript.normalize("NFD").replace(/[\u0300-\u036f]/g, "")
      var body={transcript:final_transcript}
      console.log(body)
      user_response_publisher.publish({data: JSON.stringify(body)})
    }
    return;
  }
  state="SPEAK"
  final_transcript = '';
  recognition.lang = lang;
  recognition.start();
  ignore_onend = false;
  //final_span.innerHTML = '';
  start_img.src = 'data:image/svg+xml,%3Csvg%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20viewBox%3D%220%200%2024%2024%22%3E%3Cpath%20fill%3D%22none%22%20d%3D%22M0%200h24v24H0V0z%22%2F%3E%3Cpath%20d%3D%22M10.8%204.9c0-.66.54-1.2%201.2-1.2s1.2.54%201.2%201.2l-.01%203.91L15%2010.6V5c0-1.66-1.34-3-3-3-1.54%200-2.79%201.16-2.96%202.65l1.76%201.76V4.9zM19%2011h-1.7c0%20.58-.1%201.13-.27%201.64l1.27%201.27c.44-.88.7-1.87.7-2.91zM4.41%202.86L3%204.27l6%206V11c0%201.66%201.34%203%203%203%20.23%200%20.44-.03.65-.08l1.66%201.66c-.71.33-1.5.52-2.31.52-2.76%200-5.3-2.1-5.3-5.1H5c0%203.41%202.72%206.23%206%206.72V21h2v-3.28c.91-.13%201.77-.45%202.55-.9l4.2%204.2%201.41-1.41L4.41%202.86z%22%2F%3E%3C%2Fsvg%3E';
  showInfo('info_allow');
  start_timestamp = event.timeStamp;
}

function showInfo(s) {
    // Info regarding the state of the recording
    console.log(s)
}

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

