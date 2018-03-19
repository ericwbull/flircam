#!/usr/bin/perl

use Cwd;
use File::Spec::Functions;

while (<STDIN>)
{
	$line = $_;
	undef($col);
	
    if ( $line =~ /([\.a-zA-Z0-9\/.\-_]+):([0-9]+):([0-9]+): (.*)/ )
	{
		$filePath = $1;
		$row = $2;
		$col = $3;
		$message = $4;
	}
    elsif ( $line =~ /([\.a-zA-Z0-9\/.\-_]+):([0-9]+): (.*)/ )
    {
		$filePath = $1;
		$row = $2;
		$message = $3;
	}
	else {
		print $line;
		next;
	}
	
	if ( $filePath !~ /^\// )
	{
		# prepend with the current working directory;
		$filePath = catfile(cwd(),$filePath);
	}
	
	if ( $filePath =~ /\/home\/([\.a-zA-Z0-9\/.\-_]+)/ )
	{
		$filename = $1;
	}
	else {
		print $line;
		next;
	}

	# translate slashes

	$filename =~ s#/#\\#g;

	if (defined($col))
	{
		print "\\\\raspberrypi\\$filename($row,$col): $message\n";
	}
	else 
	{
		print "\\\\raspberrypi\\$filename($row): $message\n";
	}
}
