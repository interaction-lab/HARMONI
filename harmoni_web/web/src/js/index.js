var page = "pageContent2";
//var view = "container_1";

$(document).ready(function () {

    $.getJSON("src/config/config.json", function (data) {
            $.each(data, function (key, val) {
                if (page == key) {
                    $.each(val, function (k, v) {
                        console.log(v);
                        var id = v.id;
                        var component = v.component;
                        var children = v.children;
                        var id_parent = "body_page";
                        handleComponents(children, id, component, id_parent);
                        $("#"+id).hide();
                    });
                };
            });
        })
        .done(function () {
            //$("#"+view).show();
            $("button").on("click", function () {
                clickListener(this);
            });
            $("a").on("click", function () {
                clickListener(this);
            });
        });
});

function viewListener(view){
    //Waiting for the view request from the ROS package
    console.log(view.data)
    $("#"+view.data).show();
};

function clickListener(clicked_component) {
    var selected_item = clicked_component.id;
    console.log("Clicked")
    user_response_publisher.publish({ data: selected_item })
    // Send the event clicked to the ROS package
}

function handleComponents(children, id, component, id_parent) {
    if (Array.isArray(children)) {
        var component_html = createComponent(component, children, id);
        $("#" + id_parent).append(component_html);
        $.each(children, function (k_c, v_c) {
            console.log(v_c);
            var id_c = v_c.id;
            var component_c = v_c.component;
            var children_c = v_c.children;
            handleComponents(children_c, id_c, component_c, id);
        });
    } else {
        var component_html = createComponent(component, children, id);
        $("#" + id_parent).append(component_html);
    }
}


function createComponent(component, content, id) {
    switch (component) {
        case "container":
            var html = "<div class ='container' id=" + id + "></div>";
            break;
        case "click_img":
            var html = "<a id=" + id + "><img src=" + content + "></a>";
            break;
        case "img":
            var html = "<img src=" + content + " id=" + id + "/>";
            break;
        case "text":
            if (!Array.isArray(content)) {
                var html = "<p id=" + id + ">" + content + "</p>";
            } else {
                var html = "<p id=" + id + "></p>";
            }
            break;
        case "title":
            if (!Array.isArray(content)) {
                var html = "<div class='title' id=" + id + ">" + content + "</div>";
            } else {
                var html = "<div class='title' id=" + id + "></div>";
            }
            break;
        case "button":
            var html = "<button id=" + id + ">" + content + "</button>";
            break;
        case "row":
            var html = "<div class='row' id=" + id + "></div>";
            break;
        case "col":
            var html = "<div class='col' id=" + id + "></div>";
            break;
    }
    return html;
}
