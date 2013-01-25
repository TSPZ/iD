iD.modes.Select = function(entities, initial) {
    var mode = {
        id: 'select',
        button: 'browse',
        entity: entities
    };

    var ids = _.pluck(entities, 'id');

    var inspector = iD.ui.inspector().initial(!!initial),
        keybinding = d3.keybinding('select'),
        behaviors,
        radialMenu;

    function remove() {
        entities.forEach(function(entity) {
            if (entity.type === 'way') {
                mode.history.perform(
                    iD.actions.DeleteWay(entity.id),
                    'deleted a way');
            } else if (entity.type === 'node') {
                mode.history.perform(
                    iD.actions.DeleteNode(entity.id),
                    'deleted a node');
            }
        });

        mode.controller.exit();
    }

    function changeTags(d, tags) {
        if (!_.isEqual(entities[0].tags, tags)) {
            mode.history.perform(
                iD.actions.ChangeEntityTags(d.id, tags),
                'changed tags');
        }
    }

    mode.enter = function() {
        var map = mode.map,
            graph = map.history().graph(),
            history = map.history(),
            surface = mode.map.surface;

        inspector.graph(graph);

        behaviors = [
            iD.behavior.Hover(),
            iD.behavior.DragNode(mode),
            iD.behavior.DragWay(mode),
            iD.behavior.DragMidpoint(mode)];

        behaviors.forEach(function(behavior) {
            behavior(surface);
        });

        var q = iD.util.stringQs(location.hash.substring(1));
        location.replace('#' + iD.util.qsString(_.assign(q, {
            id: ids.join(',')
        }), true));

        if (entities.length === 1) {
            d3.select('.inspector-wrap')
                .style('display', 'block')
                .style('opacity', 1)
                .datum(entities[0])
                .call(inspector);

            if (d3.event) {
                // Pan the map if the clicked feature intersects with the position
                // of the inspector
                var inspector_size = d3.select('.inspector-wrap').size(),
                    map_size = mode.map.size(),
                    offset = 50,
                    shift_left = d3.event.x - map_size[0] + inspector_size[0] + offset,
                    center = (map_size[0] / 2) + shift_left + offset;

                if (shift_left > 0 && inspector_size[1] > d3.event.y) {
                    mode.map.centerEase(mode.map.projection.invert([center, map_size[1]/2]));
                }
            }

            inspector
                .on('changeTags', changeTags)
                .on('close', function() { mode.controller.exit(); });
        }

        history.on('change.select', function() {
            // Exit mode if selected entity gets undone
            var old = entity;
            entity = history.graph().entity(entity.id);
            if (!entity) {
                mode.controller.enter(iD.modes.Browse());
            } else if(!_.isEqual(entity.tags, old.tags)) {
                inspector.tags(entity.tags);
            }

            surface.call(radialMenu.close);
        });

        map.on('move.select', function() {
            surface.call(radialMenu.close);
        });

        function click() {
            var datum = d3.select(d3.event.target).datum();
            if (datum instanceof iD.Entity) {
                mode.controller.enter(iD.modes.Select(datum));
            } else {
                mode.controller.enter(iD.modes.Browse());
            }
        }

        function dblclick() {
            var datum = d3.select(d3.event.target).datum();
            if (datum instanceof iD.Entity &&
                (datum.geometry(graph) === 'area' || datum.geometry(graph) === 'line')) {
                var choice = iD.geo.chooseIndex(datum,
                        d3.mouse(mode.map.surface.node()), mode.map),
                    node = iD.Node({ loc: choice.loc });

                history.perform(
                    iD.actions.AddNode(node),
                    iD.actions.AddWayNode(datum.id, node.id, choice.index),
                    'added a point to a road');

                d3.event.preventDefault();
                d3.event.stopPropagation();
            }
        }

        surface.on('click.select', click)
            .on('dblclick.select', dblclick);

        keybinding.on('⌫', remove);

        d3.select(document)
            .call(keybinding);

        surface.selectAll("*")
            .filter(function (d) {
                return d && entities && _.contains(ids, d.id);
            })
            .classed('selected', true);

        if (entities.length === 1) {
            radialMenu = iD.ui.RadialMenu(entities[0], history, mode.map);

            if (d3.event && !initial) {
                var loc = map.mouseCoordinates();

                if (entities[0].type === 'node') {
                    loc = entity.loc;
                }

                surface.call(radialMenu, map.projection(loc));
            }
        }
    };

    mode.exit = function () {
        var surface = mode.map.surface,
            history = mode.history;

        if (entities[0]) {
            changeTags(entity, inspector.tags());
        }

        d3.select('.inspector-wrap')
            .style('display', 'none')
            .html('');

        // Firefox incorrectly implements blur, so typeahead elements
        // are not correctly removed. Remove any stragglers manually.
        d3.selectAll('div.typeahead').remove();

        behaviors.forEach(function(behavior) {
            behavior.off(surface);
        });

        var q = iD.util.stringQs(location.hash.substring(1));
        location.replace('#' + iD.util.qsString(_.omit(q, 'id'), true));

        keybinding.off();

        surface.on('click.select', null)
            .on('dblclick.select', null);

        history.on('change.select', null);

        surface.selectAll(".selected")
            .classed('selected', false);

        surface.call(radialMenu.close);
    };

    return mode;
};
