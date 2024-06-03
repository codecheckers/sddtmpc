## Zenodo deposit; see vignette("codecheck_overview.Rmd")

library("codecheck")
library(yaml)
library(zen4R)

## This assumes your working directory is the codecheck directory

yaml_file <- "../codecheck.yml"

metadata = read_yaml(yaml_file)

## To interact with the Zenodo API, you need to create a token.  This should
## not be shared, or stored in this script.  Here I am using the Unix password
## tool pass to retrieve the token.
my_token = system("pass show codechecker-token", intern=TRUE)

## make a connection to Zenodo API
zenodo <- ZenodoManager$new(token = my_token)


if ( length(grep(pattern="zenodo.FIXME$", metadata$report)) == 1) {
  myrec <- zenodo$createEmptyRecord()
  add_id_to_yml(myrec$id, yaml_file)
  ## now re-read meta-data to check
  metadata <- read_yaml(yaml_file)  
}


record = get_zenodo_record(metadata$report)
set_zenodo_metadata1(zenodo, record, metadata)

## If you have already uploaded the certificate once, you will need to
## delete it via the web page before uploading it again.
codecheck:::set_zenodo_certificate(zenodo, record, "codecheck.pdf")

## You may also create a ZIP archive of of any data or code files that
## you think should be included in the CODECHECK's record.

## Now go to zenodo and check the record (the URL is printed
## by set_zenodo_metadata() ) and then publish.


######################################################################




## Helper functions
add_id_to_yml <- function(id, yml_file) {
  ## Add id to the yaml file.
  data1 <- readLines(yml_file)
  data2 <- gsub(pattern = "zenodo.FIXME$",
                replace = paste0("zenodo.",id),
                x = data1)
  writeLines(data2, yml_file)
}


set_zenodo_metadata1 <- function(zen, record, metadata) {
  draft <- zen$getDepositionById(record)
  if (is.null(draft)) {
    draft <- zen$getRecordById(record)
  }
  if (is.null(draft)) 
    stop("Neither deposition nor record found for ID ", record)
  draft$setResourceType("publication")

  ## Community must now be uploaded separately.

  draft$setTitle(paste("CODECHECK certificate", metadata$certificate))
  draft$metadata$languages <- NULL
  draft$addLanguage(language = "eng")
  draft$metadata$creators <- NULL
  num_creators <- length(metadata$codechecker)
  for (i in 1:num_creators) {
    draft$addCreator(name = metadata$codechecker[[i]]$name, 
                     orcid = metadata$codechecker[[i]]$ORCID)
  }
  description_text <- paste("CODECHECK certificate for paper:", 
                            metadata$paper$title)
  repo_url <- gsub("[<>]", "", metadata$repository)
  description_text <- paste(description_text,
                            sprintf("<p><p>Repository: <a href=\"%s\">%s</a>", 
                                    repo_url, repo_url))
  draft$setDescription(description_text)
  ##browser()
  draft$setSubjects(c("CODECHECK"))
  draft$setNotes(c("See file LICENSE for license of the contained code. The report document codecheck.pdf is published under CC-BY 4.0 International."))
  ##draft$setAccessRight(accessRight = "open")
  draft$metadata$rights <- NULL
  draft$setLicense("cc-by-4.0") #tocheck
  ##draft$addRelatedIdentifier(relation = "isSupplementTo", identifier = metadata$repository)
  ##draft$addRelatedIdentifier(relation = "isSupplementTo", identifier = metadata$paper$reference)
  draft <- zen$depositRecord(draft)


  zen$createReviewRequest(draft, "codecheck")
  zen$submitRecordForReview(draft$id)

  cat(paste0("Check your record online at https://zenodo.org/deposit/", 
             record, "\n"))
}



d <- zenodo$getDepositionById("11403956")

myrec <- ZenodoRecord$new()



## this works locally but does not create an id.
myrec <- ZenodoRecord$new()

myrec$setTitle("my R package")
myrec$setDescription("A description of my R package")
myrec$setResourceType("software")  ## changed
myrec$addCreator(firstname = "John", lastname = "Doe", orcid = "0000-0000-0000-0000") ## changed


## get a full list of licences by
##l <- zenodo$getLicenses()
myrec$setLicense("cc-by-4.0")

##myrec$setAccessRight("open")
myrec$setDOI("mydoi") #use this method if your DOI has been assigned elsewhere, outside Zenodo
myrec$addCommunity("ecfunded")


"cc-by-4.0" 



## test

myrec <- ZenodoRecord$new()
myrec$setTitle("my R package")
myrec$setDescription("A description of my R package")
myrec$setUploadType("software")
myrec$addCreator(firstname = "John", lastname = "Doe", affiliation = "Independent", orcid = "0000-0000-0000-0000")
myrec$setLicense("mit")
myrec$setAccessRight("open")
myrec$setDOI("mydoi") #use this method if your DOI has been assigned elsewhere, outside Zenodo
myrec$addCommunity("ecfunded")




## Setting the community.

my_community = "codecheck"
zen_com = zenodo$getCommunityById(my_community)
#retrieve the request that was created with the review submission (using status the target community and the target record)
reqs = zenodo$getRequests(q = sprintf("status:submitted AND receiver.community:%s AND topic.record:%s", zen_com$id, record$id))
reqs = zenodo$getRequests(q = sprintf("status:submitted AND receiver.community:%s", zen_com$id))




record
zenodo$createReviewRequest(record, "codecheck")
zenodo$submitRecordForReview(record$id)
