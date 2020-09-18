#!/usr/bin/perl -w
#
# Script to create a connected 2D terrain of sensors
#
# Author: Dimitrios Zorbas (jim/at/students/cs/unipi/gr)
# based on the terrain generator of D. Glynos
# Distributed under the GPLv3 (see LICENSE file)

use Graph;
use Math::Random;
use strict;

# We use decimeter precision (1/10th of a meter). No two sensors shall occupy
# the same square decimeter.

my $sensor_reading_radius = 10 * 10; 	# in deci-meters
my $sensor_comm_radius = 50 * 10; 	# in deci-meters
my %m = ();				# if the value is 1, the key node is cds node
my %nb = ();
my %nodes = ();
my %hop = ();
my @sensing_nodes = ();
my ($terrain_x, $terrain_y) = (1000 * 10, 1000 * 10); 	# 1km by 1km terrain


sub distance {
	my ($x1, $x2, $y1, $y2) = @_;
	return sqrt( (($x1-$x2)*($x1-$x2)) + (($y1-$y2)*($y1-$y2)) );
}

sub random_int {
	my $low = shift;
	my $high = shift;
	return Math::Random::random_uniform_integer(1, $low, $high);
}

sub dominant{
	my $v = shift;
	$m{$v} = 0;
	my $g = Graph::Undirected->new;
	foreach my $n (@{$nb{$v}}){
		if ($n > $v){
			$g->add_vertex($n) if (!$g->has_vertex($n));
			foreach my $n_ (@{$nb{$v}}){
				next if (($n_ eq $n) || ($n_ < $v));
				if (grep {$_ eq $n} @{$nb{$n_}}){
					$g->add_edge($n, $n_);
				}
			}
		}
	}
	if ($g->is_connected){
		my $counter = scalar @{$nb{$v}};
		my @ver = $g->vertices;
		foreach my $n (@{$nb{$v}}){
			my $init_c = $counter;
			if (grep {$_ eq $n} @ver){
				$counter -= 1;
			}
			foreach my $vn (@ver){
				if (($counter == $init_c) && (grep {$_ eq $n} @{$nb{$vn}})){
					$counter -= 1;
				}
			}
		}
		if ($counter != 0){
			$m{$v} = 1;
		}
	}else{
		$m{$v} = 1;
	}
}

(@ARGV==2) || die "usage: $0 <num_of_nodes> <sensing_nodes%>\n";

my $num_nodes = $ARGV[0];
my $sensing_nodes = int($ARGV[1]*$num_nodes/100);

if ($sensing_nodes == 0){
	$sensing_nodes = 1;
}

my $density = 0.14142136;
my $norm_x = int($terrain_x * $density);	# normalised terrain_x
my $norm_y = int($terrain_y * $density); 	# normalised terrain_y

my $base_x = 1;		# x location of base node
my $base_y = int($norm_y / 2);	# y location of base node


### GENERATING SENSORS ###

my %nodes_temp = ();
for(my $i=1; $i<=$num_nodes; $i++){
	my ($x,$y) = (random_int(1, $norm_x), random_int(1, $norm_y));

	while (exists $nodes_temp{$x}{$y}){
		($x, $y) = (random_int(1, $norm_x), random_int(1, $norm_y));
	}
	$nodes_temp{$x}{$y} = 1;
	$nodes{$i} = [$x, $y];
	if ($i <= $sensing_nodes){
		push (@sensing_nodes, $i);
	}
}


### COMPUTE NEIGHBOR SETS ###

$nodes{"0"} = [$base_x, $base_y];
foreach my $n (keys %nodes){
	my ($x, $y) = @{$nodes{$n}};
	foreach my $n_ (keys %nodes){
		next if ($n_ eq $n);
		my ($x_, $y_) = @{$nodes{$n_}};
		if (distance($x, $x_, $y, $y_) <= $sensor_comm_radius){
			push (@{$nb{$n}}, $n_);
		}
	}
}


### COMPUTE CDS NODES ###

foreach my $n (keys %nodes){
	dominant($n);
}
$m{"0"} = 1;


### COMPUTE GRAPH ###

my $graph = Graph::Undirected->new;
foreach my $n (keys %m){
	next if (($m{$n} == 0) || ($n eq "0"));
	my ($x, $y) = @{$nodes{$n}};
	foreach my $n_ (keys %m){
		next if (($m{$n_} == 0) || ($n_ eq $n));
		my ($x_, $y_) = @{$nodes{$n_}};
		if (distance($x, $x_, $y, $y_) <= $sensor_comm_radius){
			$graph->add_edge($n, $n_) if (!$graph->has_edge($n, $n_));
		}
	}
}

foreach my $n (@sensing_nodes){
	my ($x, $y) = @{$nodes{$n}};
	my $min = $sensor_comm_radius + 1;
	my $sel = undef;
	foreach my $n_ (keys %m){
		next if (($m{$n_} == 0) || ($n_ eq $n));
		my ($x_, $y_) = @{$nodes{$n_}};
		if ((grep {$_ eq $n_} @{$nb{$n}}) && (distance($x, $x_, $y, $y_) < $min)){
			$min = distance($x, $x_, $y, $y_);
			$sel = $n_;
		}
	}
	$graph->add_edge($n, $sel);
}


### DELETE UNUSED CDS NODES ###

my @used_cds = ();
foreach my $n (@sensing_nodes){
	my @path = $graph->SP_Dijkstra($n, "0");
	if (scalar @path == 0){
		printf "# Non-connected cds graph!\n";
		exit 1;
	}
	$hop{$n} = (scalar @path) - 1;
	while (scalar @path > 0){
		my $s = shift(@path);
		if ($s ne $n){
			if (!(grep {$_ eq $s} @used_cds)){
				push (@used_cds, $s);
			}
		}
	}
}
foreach my $n (keys %m){
	next if ($m{$n} == 0);
	if ((!(grep {$_ eq $n} @used_cds)) && (!(grep {$_ eq $n} @sensing_nodes))){
		$graph->delete_vertex($n);
		$m{$n} = 0;
	}else{
		my @path = $graph->SP_Dijkstra($n, "0");
		$hop{$n} = (scalar @path) - 1;
	}
}
my $sptg = $graph->SPT_Dijkstra("0");

### PRINT OUTPUT ###

printf "# terrain map [%i x %i]\n", $norm_x, $norm_y;

print "# sensor coords:";
foreach my $n (keys %nodes){
	my ($x, $y) = @{$nodes{$n}};
	printf " %i [%i %i]", $n, $x, $y;
}
print "\n";

print "# cds nodes:";
foreach my $n (keys %m){
	next if ($m{$n} == 0);
	print " $n [$hop{$n}]";
}
print "\n";

print "# sensing nodes:";
foreach my $n (@sensing_nodes){
	print " $n [$hop{$n}]";
}
print "\n";

printf "# base station coords: [%i %i]\n", $base_x, $base_y;
print "# generated with: $0 ",join(" ",@ARGV),"\n";
printf "# stats: sensors=%i sensing_nodes=%d terrain=%.1fm^2 sensor_sz=%.2fm^2 sensor_comm_radius=%.2fm\n", $num_nodes, $sensing_nodes, ($norm_x * $norm_y) / 100, 0.1 * 0.1, $sensor_comm_radius / 10;
printf "# Graph: %s\n", $sptg;
printf "# %s\n", '$Id: generate_terrain.pl 1881 2012-06-10 19:22:25Z jim $';
exit 0;
