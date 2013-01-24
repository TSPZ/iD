iD.ui.RadialMenu = function(entity, history) {
    var radialMenu = function(selection, center) {
        var operations,
            graph = history.graph(),
            geometry = entity.geometry(graph);

        if (geometry === 'vertex') {
            operations = [
                {
                    id: 'delete',
                    text: 'Delete',
                    description: 'deleted a node',
                    action: iD.actions.DeleteNode(entity.id)
                },
                {
                    id: 'split',
                    text: 'Split Way',
                    description: 'split a way',
                    action: iD.actions.SplitWay(entity.id)
                },
                {
                    id: 'unjoin',
                    text: 'Unjoin',
                    description: 'unjoined lines',
                    action: iD.actions.UnjoinNode(entity.id)
                }
            ];
        } else if (geometry === 'point') {
            operations = [
                {
                    id: 'delete',
                    text: 'Delete',
                    description: 'deleted a point',
                    action: iD.actions.DeleteNode(entity.id)
                }
            ];
        } else if (geometry === 'line') {
            operations = [
                {
                    id: 'delete',
                    text: 'Delete',
                    description: 'deleted a line',
                    action: iD.actions.DeleteWay(entity.id)
                },
                {
                    id: 'reverse',
                    text: 'Reverse',
                    description: 'reversed a way',
                    action: iD.actions.ReverseWay(entity.id)
                }
            ];
        } else if (geometry === 'area') {
            operations = [{
                    id: 'delete',
                    text: 'Delete',
                    description: 'deleted an area',
                    action: iD.actions.DeleteWay(entity.id)
            }, {
                    id: 'orthogonalize',
                    text: 'Orthogonalize',
                    description: 'orthogonalized an area',
                    action: iD.actions.Orthogonalize(entity.id)
            }];
        }

        var arc = d3.svg.arc()
            .outerRadius(70)
            .innerRadius(30)
            .startAngle(function (d, i) { return 2 * Math.PI / operations.length * i; })
            .endAngle(function (d, i) { return 2 * Math.PI / operations.length * (i + 1); });

        var arcs = selection.selectAll('.arc-menu')
            .data(operations)
          .enter().append('g')
            .attr('class', 'arc-menu')
            .attr('transform', "translate(" + center + ")")
            .attr('opacity', 0);

        arcs.transition()
            .attr('opacity', 0.8);

        arcs.append('path')
            .attr('class', function (d) { return 'arc-menu-item arc-menu-item-' + d.id; })
            .attr('d', arc)
            .classed('disabled', function (d) { return !d.action.enabled(history.graph()); })
            .on('click', function (d) { history.perform(d.action, d.description); });

        arcs.append('text')
            .attr("transform", function(d, i) { return "translate(" + arc.centroid(d, i) + ")"; })
            .attr("dy", ".35em")
            .style("text-anchor", "middle")
            .text(function(d) { return d.text; });
    };

    radialMenu.close = function(selection) {
        selection.selectAll('.arc-menu')
            .remove();
    };

    return radialMenu;
};
