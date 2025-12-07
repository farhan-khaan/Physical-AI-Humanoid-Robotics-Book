// Docusaurus configuration for environment variables
// This file allows accessing process.env variables in the client

module.exports = {
  // ... other config
  customFields: {
    // Make API URL available to client-side code
    apiUrl: process.env.REACT_APP_API_URL || 'http://localhost:8000',
  },
};
